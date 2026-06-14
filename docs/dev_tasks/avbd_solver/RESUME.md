# Resume: AVBD Solver

## Current Resume Handoff (2026-06-14)

The user resumed after the previous literal-stop checkpoint by saying
`continue`. This section is the current handoff snapshot and supersedes the
stop-only snapshot lower in this file.

Current local follow-up branch state: `avbd/soft-body-inertia-orientation-cache`
is an unpublished follow-up branch based on PR #2991 head `6f41e9529bf`. It
currently contains the local 3D Spring/Spring Ratio ignored-collision-pair
slice plus the previous 2D Spring/Spring Ratio contact-filtering slice,
inertia-orientation cleanup, and handoff-doc refreshes. The latest kept
implementation explicitly ignores spring-connected rigid body collision pairs
in the 3D Python source rows and matching C++ benchmark constructors. Do not
push, open a PR, resolve review threads, comment, retrigger reviews, or mutate
CI without explicit maintainer approval.

Current verified checkpoint PR state: after explicit maintainer approval, PR
#2991 branch `avbd/source-row-extraction-precheck` was pushed to origin and is
currently at `6f41e9529bf` after verifying latest `origin/main` was already
merged. The previous three CI-fix commits, the AVBD contact-config guard
follow-up, and the handoff-doc refresh are on the PR branch, hosted CI is still
running, and the branch is mergeable with no failed checks observed in the
latest rollup. Review-thread resolution, PR comments, review re-triggers, and
any further push still require explicit maintainer approval.

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Keep
claims narrow. Do not claim a paper/source-demo CPU win, GPU parity, broad
breakable-wall/fracture corpus, same-hardware paper-number match, or
all-coefficient friction win unless the tracked artifacts directly prove it.

Latest local slice: the 3D Spring and Spring Ratio source rows now explicitly
ignore collision pairs that are already connected by the measured spring
constraints. The C++ `BM_AvbdDemo3dSpringStep` and
`BM_AvbdDemo3dSpringRatioStep` benchmark constructors mirror those ignored
pairs and expose `ignored_collision_pairs` counters. Focused Python coverage
asserts the 3D Spring row has one ignored pair and the 3D Spring Ratio row has
seven adjacent ignored pairs, and also verifies a non-adjacent 3D Spring Ratio
pair remains collidable. The benchmark smoke is configuration/path evidence
only under high local load; it does not refresh a tracked packet, close the 3D
Spring/Spring Ratio CPU gates, push a follow-up PR, resolve GitHub review
threads, or claim GPU/paper-number parity.

Previous local slice: the 2D Spring and Spring Ratio source rows now explicitly
ignore collision pairs that are already connected by the measured spring
constraints. The C++ `BM_AvbdDemo2dSpringStep` and
`BM_AvbdDemo2dSpringRatioStep` benchmark constructors mirror those ignored
pairs and expose `ignored_collision_pairs` counters. The rigid body contact
stage keeps the old fast path for worlds without ignored pairs, audits small
rigid-body candidate sets when ignored pairs exist, and skips collision query
work only when every dynamic-involving candidate pair is explicitly ignored.
This is deliberately bounded to small worlds; larger candidate sets fall back
to the ordinary collision query path. Focused Python coverage asserts the
Spring row has one ignored pair and Spring Ratio has seven adjacent ignored
pairs, and also verifies a non-adjacent Spring Ratio pair remains collidable.
This local branch does not refresh a tracked packet, close a CPU-win gate, push
a follow-up PR, resolve GitHub review threads, or claim GPU/paper-number
parity.

Previous local slice: `addAvbdRigidBodyInertiaTermLowerTriangle()` now reuses
the already normalized current orientation and normalizes the inertial target
once before computing the rigid body orientation error, avoiding repeated
normalization in the hot block-descent body assembly path. The public robust
orientation-error helpers still normalize their inputs. Focused regression
coverage compares the lower-triangle inertia helper against the full symmetric
helper for scaled current and target quaternions. Validation passed the target
rebuild, the new focused test, the full `test_avbd_rigid_block` suite (100
tests), `pixi run lint`, `pixi run build`, and `pixi run test-unit` (161
tests). Same-session benchmark smoke rebuilt PR head `6f41e9529bf` and this
local branch under similar load: PR head recorded
`BM_AvbdDemo2dSoftBodyStep` at 1.810 ms median CPU, while the local inertia
cleanup recorded 1.792 ms median CPU. Treat that as narrow smoke only because
CPU scaling was enabled and tracked source/native packets were not regenerated.
A broader normalized-orientation reuse probe for assembly world-point/angular
row helpers was also built and tested, but it measured 1.821 ms median CPU
versus the 1.792 ms current-branch baseline, so the probe was reverted and no
code from it is kept.

Latest pushed PR slice: the rigid contact stage now skips the per-contact
`rigidAvbdContactStageConfig()` scan unless the registry has
`RigidAvbdContactConfig` storage. Default sequential-impulse worlds such as the
frictionless Dynamic Friction source row already queried contacts without AVBD
shape details, so this removes one remaining AVBD opt-in check from the
ordinary contact path while preserving the all-or-nothing AVBD config behavior
when any contact config storage exists. This does not refresh the tracked
friction-sweep packet, close the frictionless source-row CPU gap, resolve the
GitHub review threads, or claim GPU parity.

Previous local slice: the default sequential-impulse rigid contact friction solve
now returns when a clamped tangent impulse delta is exactly zero and skips
static/prescribed endpoint velocity writes for tangent impulses. This trims
no-op work in the same Dynamic Friction coefficient sweep contact path without
changing the normal solve or static endpoint semantics. It does not refresh the
tracked friction-sweep packet, close the frictionless source-row CPU gap, or
claim GPU parity.

Prior local slice: the default sequential-impulse rigid contact assembly now
shares each endpoint's `ContactMaterial` lookup between friction and
restitution and skips `sqrt` for exact zero combined friction products. This is
a narrow fixed-cost cleanup for the
`BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row. It does not
refresh the tracked friction-sweep packet, close the frictionless source-row
CPU gap, or claim GPU parity.

Review follow-up addressed on the pushed branch: a read-only GitHub thread
refresh still shows two unresolved Codex bot threads on PR #2991. The older
numeric-equivalent friction-env thread is covered by the focused packet writer
regression. The newer review `4492237805` thread noted that the Dynamic
Friction panel still plotted `Friction 5 speed` when
`DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION` changed the maximum friction.
The panel label now derives from `max_friction`, the value is recorded in
`SceneSetup.info`, and the focused integration test builds the panel under the
`2.5` override and asserts that `Friction 2.5 speed` replaces the stale
`Friction 5 speed` label. No GitHub reply, thread resolution, or review
retrigger has been performed.

Merge state: `origin/main` at `c4c5ed87eae4` was merged after the review-fix
commit as `5c43ed6f843`; the only manual conflict was
`scripts/capture_py_demo.py`, resolved by preserving this branch's
`--env`/`--metadata` capture overrides and `main`'s visual-verification
resolved-solver-identity / scene-metrics manifest support. The branch was then
updated again with latest `origin/main` at `9de4ac6af87` before the approved
push, committed as `fffb5bf45ba`.

Hosted CI follow-up: pushed head `fffb5bf45ba` exposed a `DART GUI Smoke
(Clang)` failure in the CI Linux run before ordinary compilation. CMake 4.3
emitted C++20 module dependency-scanning rules that invoked missing
`clang-scan-deps` even though DART has no C++ module sources. The local
follow-up disables `CMAKE_CXX_SCAN_FOR_MODULES` at the root project and adds a
changelog note. `pixi run lint`, `pixi run build`, and the post-push
`pixi run test-unit` passed. A fresh Clang/Ninja smoke configure without
`clang-scan-deps` generated no `.ddi`, `clang-scan-deps`, or
`CMAKE_CXX_COMPILER_CLANG_SCAN_DEPS-NOTFOUND` Ninja rules and reached normal
`.o` compilation for `dart-collision-native`; local Clang 22 then failed on
missing standard C++ header `cmath`, which is a local toolchain issue rather
than the hosted scanner failure. This CI fix was pushed as part of PR head
`0bf4ca6b8ae`.

Second CI-smoke follow-up now pushed: while validating the GUI smoke path locally,
`pixi run test-dart-gui-smoke` initially failed every generated smoke test
because `scripts/run_cpp_example.py` prepended a relative build library path to
`LD_LIBRARY_PATH`; each CTest smoke test runs from a nested working directory,
so the loader could fall back to an installed DART library that required
unavailable `libfcl.so.0.7`. The runner now prepends the absolute
build-library path, and `pixi run test-dart-gui-smoke` passes 31/31 smoke
tests. Focused regression coverage now passes
`pixi run pytest python/tests/unit/test_run_cpp_example.py::test_runtime_env_prepends_absolute_build_library_path -q`,
the full `python/tests/unit/test_run_cpp_example.py` file passes, and
`pixi run lint` passes. This CI-smoke fix and regression were pushed as part of
PR head `0bf4ca6b8ae`.

Validation for the latest 3D Spring/Spring Ratio ignored-pair slice:

- `pixi run -- cmake --build build/default/cpp/Release --target bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run env PYTHONPATH=build/default/cpp/Release/python:python pytest python/tests/integration/test_demos_cycle.py::test_avbd_demo3d_spring_scene_matches_source_row python/tests/integration/test_demos_cycle.py::test_avbd_demo3d_spring_ratio_scene_matches_source_row -q`
  passed, 2 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo3d(SpringStep|SpringRatioStep)$" --benchmark_min_time=0.3s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-local-3d-spring-contact-filter-20260614.json --benchmark_out_format=json'`
  passed. Under load average `22.04, 11.41, 8.44` with CPU scaling enabled,
  it recorded median CPU step times of 9.880 us for
  `BM_AvbdDemo3dSpringStep` with `ignored_collision_pairs=1` and 78.036 us
  for `BM_AvbdDemo3dSpringRatioStep` with `ignored_collision_pairs=7`. This is
  source-row configuration/path smoke only because same-source native timing,
  visual captures, and packet regeneration were not rerun.
- `git diff --check` passed.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The final report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The simulation
  label passed 73/73 tests in 963.42 s, including
  `test_rigid_ipc_paper_experiments` at 578.36 s and
  `test_lcp_jacobi_batch_cuda` at 207.77 s. The CUDA-specific label passed 8/8
  tests in 210.65 s, with `test_lcp_jacobi_batch_cuda` at 207.54 s. The docs
  build passed with the existing four `dartpy._world_render_bridge` autodoc
  import warnings, and CUDA benchmark smokes passed under CPU-scaling warnings
  and load averages around 8-10, so treat those timings as smoke evidence only.

Validation for the latest Spring/Spring Ratio ignored-pair slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.SimulationCollisionQueryCanIgnoreSpecificPairs:World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests.
- `pixi run env PYTHONPATH=build/default/cpp/Release/python:python pytest python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_spring_scene_matches_source_row python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_spring_ratio_scene_matches_source_row -q`
  passed, 2 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2d(SpringStep|SpringRatioStep)$" --benchmark_min_time=0.3s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-local-spring-contact-filter-final-20260614.json --benchmark_out_format=json'`
  passed. Under load average `2.90, 5.92, 8.45` with CPU scaling enabled, it
  recorded median CPU step times of 3.313 us for
  `BM_AvbdDemo2dSpringStep` with `ignored_collision_pairs=1` and 34.588 us
  for `BM_AvbdDemo2dSpringRatioStep` with `ignored_collision_pairs=7`. This is
  source-row path smoke only because same-source native timing, visual
  captures, and packet regeneration were not rerun.
- `git diff --check` passed.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The final CUDA
  simulation label passed 8/8 tests; its slowest test was
  `test_lcp_jacobi_batch_cuda` at 208.20 s. Final CUDA benchmark smokes passed
  under CPU-scaling warnings and load averages around 15.70-16.38, so treat
  those timings as smoke evidence only.

Validation for the latest AVBD config-guard slice:

- `DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION=0 PYTHONPATH=build/default/cpp/Release/python:python LD_LIBRARY_PATH=build/default/cpp/Release/lib:.pixi/envs/default/lib pixi run python - <<'PY' ...`
  profiled the Python `avbd_demo2d_dynamic_friction` source-shaped scene for
  eight steps. The last-step profile reported wall time 0.044 ms with
  `rigid_body_contact` at 0.034 ms / 77.2% wall, so the frictionless row
  remains contact-stage dominated.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-current-head-20260614.json --benchmark_out_format=json'`
  passed before this edit. It recorded a 7.837 us median CPU step under load
  average `8.72, 12.78, 12.68` with CPU scaling enabled.
- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere' --gtest_brief=1`
  passed, 3 tests.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactStageAvbd*' --gtest_brief=1`
  passed, 44 tests.
- `pixi run env PYTHONPATH=build/default/cpp/Release/python:python pytest python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_dynamic_friction_scene_max_friction_env python/tests/unit/test_write_avbd_friction_coefficient_sweep_packet.py::test_avbd_friction_coefficient_sweep_packet_accepts_equivalent_capture_env -q`
  passed, 2 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-avbd-config-guard-20260614.json --benchmark_out_format=json'`
  passed after the edit. It recorded a 7.409 us median CPU step under load
  average `4.06, 6.11, 9.52` with CPU scaling enabled. This is path smoke only
  because same-source native timing, visual captures, and packet regeneration
  were not rerun.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.

Validation for the latest friction tangent no-op slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactRestitution:World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 6 tests in 54.68 s.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-friction-sweep-after-static-friction-write-guard-20260614.json --benchmark_out_format=json'`
  passed. It recorded median CPU step times of 7.107, 11.982, 15.833, 7.754,
  and 7.563 us for max friction 0, 0.5, 1.0, 2.5, and 5.0 under load average
  `3.61, 4.55, 7.00` with CPU scaling enabled. This is path smoke only because
  same-source native timing, visual captures, and packet regeneration were not
  rerun.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.

Rejected local probe reverted before this handoff:

- A zero-restitution guard avoided pre-solve approach-velocity calculation when
  both materials had zero restitution. Focused contact/restitution/no-allocation
  tests passed, but `/0` benchmark smokes reran at 13.71-13.76 us medians, so
  the probe was rejected and reverted before this kept slice.

Validation for the prior material-lookup slice:

- `DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION=0 PYTHONPATH=build/default/cpp/Release/python:python LD_LIBRARY_PATH=build/default/cpp/Release/lib:.pixi/envs/default/lib pixi run python - <<'PY' ...`
  profiled the Python `avbd_demo2d_dynamic_friction` source-shaped scene for
  eight steps. The last-step profile reported wall time 0.063 ms with
  `rigid_body_contact` at 0.050 ms / 80.0% wall, so the frictionless row
  remains contact-stage dominated.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-current-tip-20260614.json --benchmark_out_format=json'`
  passed before the material-lookup edit. It recorded a 7.924 us median CPU
  step under load average `14.91, 22.91, 24.88` with CPU scaling enabled.
- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests in 85.20 s.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-after-material-lookup-20260614.json --benchmark_out_format=json'`
  passed after the edit. It recorded a 7.891 us median CPU step under load
  average `22.03, 25.22, 25.49` with CPU scaling enabled. This is path smoke
  only because same-source native timing, visual captures, and packet
  regeneration were not rerun.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The run
  overlapped other local DART/CUDA work; slow stages included
  `test_rigid_ipc_paper_experiments` at 625.34 s, `test_world` at 312.14 s,
  the simulation-label `test_lcp_jacobi_batch_cuda` at 464.43 s, and the final
  CUDA smoke `test_lcp_jacobi_batch_cuda` at 381.38 s. The documentation stage
  retained the existing four `dartpy._world_render_bridge` autodoc warnings but
  passed. Final CUDA benchmark smokes passed under CPU-scaling warnings and
  load averages around 5.50-11.65, so treat those timings as smoke evidence
  only.

Validation for the prior static-endpoint slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-after-static-arm-skip-20260614.json --benchmark_out_format=json'`
  passed after this kept edit. It recorded a 7.04 us median CPU step under
  load average `1.84, 2.00, 1.34` with CPU scaling enabled and 1.22% CV. This
  is path smoke only because same-source native timing, visual captures, and
  packet regeneration were not rerun.
- `PYTHONPATH=build/default/cpp/Release/python LD_LIBRARY_PATH=build/default/cpp/Release/lib:.pixi/envs/default/lib pixi run pytest python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_dynamic_friction_scene_max_friction_env -q`
  passed, 1 test. The explicit `PYTHONPATH`/`LD_LIBRARY_PATH` were required
  because the ambient dartpy import path did not resolve the in-tree build's
  runtime libraries; the ambient command failed before collection with
  `cannot import name '_dartpy'`.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `git diff --check` passed.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The run
  overlapped other DART/CUDA work on the machine; slow stages included
  `test_rigid_ipc_paper_experiments` at 560.12 s, `test_world` at 340.00 s,
  the simulation-label `test_lcp_jacobi_batch_cuda` at 223.85 s, and the final
  CUDA smoke `test_lcp_jacobi_batch_cuda` at 227.23 s. The documentation stage
  emitted the existing four autodoc warnings about the generated
  `dartpy._world_render_bridge` stub import, but the command passed. Final
  CUDA benchmark smokes also passed under CPU-scaling warnings and load
  averages around 20-26, so treat those timings as smoke evidence only.

Post-merge validation after resolving `origin/main`:

- `pixi run python -m py_compile scripts/capture_py_demo.py` passed.
- `PYTHONPATH=build/default/cpp/Release/python LD_LIBRARY_PATH=build/default/cpp/Release/lib:.pixi/envs/default/lib pixi run pytest python/tests/unit/test_capture_py_demo.py -q`
  passed, 76 tests. The ambient command without the explicit build
  `PYTHONPATH` failed only on
  `test_visual_capture_default_scene_matches_py_demos_front_door` when it
  imported demo scenes and could not resolve the in-tree `dartpy._dartpy`
  extension.
- `PYTHONPATH=build/default/cpp/Release/python LD_LIBRARY_PATH=build/default/cpp/Release/lib:.pixi/envs/default/lib pixi run pytest python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_dynamic_friction_scene_max_friction_env -q`
  passed, 1 test.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests in 116.58 s.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run lint` passed.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The run
  overlapped other local DART/CUDA work; slow stages included
  `test_rigid_ipc_paper_experiments` at 476.51 s, `test_world` at 239.15 s,
  the simulation-label `test_lcp_jacobi_batch_cuda` at 379.81 s, and the final
  CUDA smoke `test_lcp_jacobi_batch_cuda` at 380.93 s. The documentation stage
  retained the existing four `dartpy._world_render_bridge` autodoc warnings but
  passed.
- `pixi run pytest python/tests/unit/test_write_avbd_friction_coefficient_sweep_packet.py::test_avbd_friction_coefficient_sweep_packet_accepts_equivalent_capture_env -q`
  passed, 1 test. This keeps the older numeric-equivalent env Codex review
  thread covered locally without replying to the bot.

Previous static-position checkpoint validation, before this local slice, is
retained for context:

- `DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION=0 PYTHONPATH=build/default/cpp/Release/python LD_LIBRARY_PATH=build/default/cpp/Release/lib:.pixi/envs/default/lib pixi run python - <<'PY' ...`
  profiled the Python `avbd_demo2d_dynamic_friction` source-shaped scene after
  five steps. The last-step profile reported wall time 0.033 ms with
  `rigid_body_contact` at 0.027 ms / 80.5% wall, so the frictionless row remains
  contact-stage dominated. The explicit `PYTHONPATH`/`LD_LIBRARY_PATH` were
  required because the ambient dartpy import path did not resolve the in-tree
  build's runtime libraries.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-current-20260613.json --benchmark_out_format=json'`
  passed before the static-position checkpoint. It recorded a 7.62 us median
  CPU step under load average `7.06, 7.73, 8.81` with CPU scaling enabled and
  10.3% CV.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-after-static-position-write-guard.json --benchmark_out_format=json'`
  passed after the static-position checkpoint. It recorded a 7.78 us median
  CPU step under load average `19.85, 12.52, 9.51` with CPU scaling enabled
  and 2.9% CV. This is path smoke only because same-source native timing,
  visual captures, and packet regeneration were not rerun, and the host was
  busy with other DART builds.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The run
  overlapped external DART/CUDA load; slow stages included
  `test_rigid_ipc_paper_experiments` at 468.77 s, `test_world` at 254.32 s,
  the simulation-label `test_lcp_jacobi_batch_cuda` at 519.43 s, and the final
  CUDA smoke `test_lcp_jacobi_batch_cuda` at 395.53 s. The documentation stage
  emitted four existing autodoc warnings about the generated
  `dartpy._world_render_bridge` stub import, but the command passed. Final CUDA
  benchmark smokes also passed under CPU-scaling warnings and load averages
  around 9-11, so treat those timings as smoke evidence only.

Rejected local probes that were reverted before this handoff:

- A normal-only convergence-exit prototype returned whether any normal impulse
  changed and broke after a no-change sweep. Focused tests passed, but `/0`
  benchmark smoke regressed to 14.44-14.75 us medians under CPU scaling.
- An independent single-normal-sweep prototype detected one dynamic body
  against one prescribed body with no angular normal term. The full-constraint
  scan version measured 7.81 us median and the scratch-list version regressed
  to 14.10 us median under CPU scaling. Neither was kept.

Next preferred bounded work: avoid more normal-iteration-control changes unless
fresh profiling points there. The remaining `/0` gap is still contact-stage
dominated; inspect collision query / contact assembly fixed costs, or switch to
GPU parity preparation if no safe CPU cut remains under cleaner host load.

Previous local checkpoint: commit `2d4357176b4` records the default
sequential-impulse normal contact solve recording whether each contact side has
a nonzero normal angular Jacobian term, skipping the corresponding zero-angular
approach-velocity and effective-mass work, avoiding no-op angular velocity
updates for centered normal rows, and returning early from normal-impulse
application when the clamped impulse delta is zero.

Validation for commit `2d4357176b4`:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-before-guarded-normal-updates.json --benchmark_out_format=json'`
  passed before the edit. It recorded a 7.96 us median CPU step under load
  average `5.59, 6.12, 8.44` with CPU scaling enabled.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-after-guarded-normal-updates.json --benchmark_out_format=json'`
  passed after the edit. It recorded a 7.49 us median CPU step under load
  average `3.01, 4.24, 6.99` with CPU scaling enabled. This is local path
  smoke only because same-source native timing and visual captures were not
  rerun.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The report passed all seven categories: linting, build, unit tests,
  simulation tests, Python tests, documentation, and CUDA tests. The run
  overlapped external DART/CUDA load; slow stages included
  `test_rigid_ipc_paper_experiments` at 396.33 s, `test_world` at 234.02 s,
  the simulation-label `test_lcp_jacobi_batch_cuda` at 539.94 s, and the final
  CUDA smoke `test_lcp_jacobi_batch_cuda` at 295.82 s. The documentation stage
  emitted four existing autodoc warnings about the generated
  `dartpy._world_render_bridge` stub import, but the command passed.

Previous local checkpoint: commit `5101bd44a92` records the default
sequential-impulse normal contact solve storing each contact's `arm x normal`
angular Jacobian terms and reusing them for normal effective mass, restitution
approach, and per-iteration normal approach velocity. This avoids rebuilding
full contact-point velocities with angular cross products inside the normal
loop. The frictional tangent solve still uses the existing full contact-point
velocity path. This targets the `BM_AvbdDemo2dFrictionCoefficientSweep/0`
source-shaped row, where the centered frictionless contacts have zero angular
normal Jacobian terms. This does not refresh the tracked friction-sweep packet,
close the frictionless source-row CPU gap, or claim GPU parity.

Validation for commit `5101bd44a92`:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-before-normal-approach-cache.json --benchmark_out_format=json'`
  passed before the edit. It recorded a 20.84 us median CPU step under load
  average `23.49, 18.35, 13.20` with CPU scaling enabled.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-after-normal-approach-cache.json --benchmark_out_format=json'`
  passed after the edit. It recorded a 20.27 us median CPU step under load
  average `25.51, 23.40, 16.85` with CPU scaling enabled. This is local path
  smoke only because same-source native timing and visual captures were not
  rerun, and the host was busy with another CUDA test-all.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The run passed lint, Release and Debug builds, 207 non-simulation C++ tests,
  69 simulation C++ tests, Python tests, docs, and CUDA test/benchmark smoke.
  The simulation stage was slow under concurrent host load
  (`test_rigid_ipc_paper_experiments` 645.92 s, `test_world` 187.55 s,
  `test_lcp_jacobi_batch_cuda` 438.38 s), and the final CUDA smoke reran
  `test_lcp_jacobi_batch_cuda` in 768.61 s before passing benchmark smoke. The
  documentation stage emitted four existing autodoc warnings about the
  generated `dartpy._world_render_bridge` stub import, but the command passed.

Previous local checkpoint: commit `2cfdc455ee3` records the default
sequential-impulse rigid contact path skipping inverse world-inertia
factorization for centered, frictionless normal contacts whose angular contact
Jacobian is exactly zero. Frictional contacts and off-center normal contacts
still compute inverse world inertia through the existing path. This targets the
`BM_AvbdDemo2dFrictionCoefficientSweep/0` source-shaped row, where the 11
zero-friction boxes contact the ground through centered normal rows. This does
not refresh the tracked friction-sweep packet, close the frictionless
source-row CPU gap, or claim GPU parity.

Validation for commit `2cfdc455ee3`:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed, 5 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-frictionless-current-profile-baseline.json --benchmark_out_format=json'`
  passed before the edit. It recorded an 8.34 us median CPU step under load
  average `4.48, 5.04, 7.18` with CPU scaling enabled.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-sweep-row-after-central-inertia-skip.json --benchmark_out_format=json'`
  passed after the edit. It recorded a 7.42 us median CPU step under load
  average `3.64, 4.33, 6.29` with CPU scaling enabled. This is local path smoke
  only because same-source native timing and visual captures were not rerun.
- A text `WorldStepProfile` snapshot on the Python
  `avbd_demo2d_dynamic_friction` scene with max friction 0 recorded
  `rigid_body_contact` at 0.027 ms after the edit; a pre-edit snapshot in the
  same session recorded 0.060 ms. Profiling overhead and host load make this
  diagnostic path evidence, not tracked packet evidence.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The documentation stage emitted four existing autodoc warnings about the
  generated `dartpy._world_render_bridge` stub import, but the command passed.

Previous local checkpoint: commit `b1e7402d1fa` records simulation-bake cache
prewarming without prepare-time contact row generation. `RigidBodyContactStage`
sizes ordinary and AVBD contact scratch from the conservative collision-shape
capacity estimate and prewarms collision-query cache storage without generating
prepare-time contacts; `execute()` remains the only path that assembles contact
rows for the actual solve. This removes duplicate bake-time contact generation
only; it does not refresh the tracked friction-sweep packet, close the
frictionless source-row CPU gap, or claim GPU parity.

Validation for commit `b1e7402d1fa`:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world -j 8`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.RigidAvbdContactRowsAreActive:World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep:World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep:World.RigidBodyContactZeroFrictionPreservesSlidingVelocity' --gtest_brief=1`
  passed, 6 tests.
- `pixi run -e cuda -- cmake --build build/cuda/cpp/Release --target test_world -j 8`
  passed.
- `pixi run -e cuda -- build/cuda/cpp/Release/bin/test_world --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1`
  passed after a first full `pixi run -e cuda test-all` attempt exposed the
  missing collision-query-cache prewarm in that same allocator regression.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161 tests.
- `pixi run -e cuda test-all` passed on the visible NVIDIA RTX 5000 Ada host.
  The documentation stage emitted four existing autodoc warnings about the
  generated `dartpy._world_render_bridge` stub import, but the command passed.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-sweep-row-after-cache-prewarm.json --benchmark_out_format=json'`
  passed. It recorded an 8.73 us median CPU step under load average
  `12.97, 10.30, 8.72` with CPU scaling enabled. This is path smoke only
  because the edit affects simulation bake, not the steady-state per-step
  contact solve.

Previous local checkpoint: commit `4c2cc43073b` records the
friction-coefficient sweep owner row matching the tracked packet evidence. The
packet records DART faster than the same-source native runner at max friction
0.5, 1.0, 2.5, and 5.0, but still slower at max friction 0.0; the
all-coefficient CPU-win gate remains open. The packet-writer regression covers
mixed faster/slower reference sweeps so the `all_dart_faster_than_reference` flag
and remaining-gate text stay true to the per-coefficient comparison. This is
evidence hygiene only; it does not refresh the benchmark packet, close the
frictionless source-row CPU gap, or claim GPU parity.

Validation for commit `4c2cc43073b`:

- `pixi run pytest python/tests/unit/test_write_avbd_friction_coefficient_sweep_packet.py`
  passed, 15 tests.
- `pixi run lint` passed.

Previous local checkpoint: commit `7a4c148352a` records the default
sequential-impulse rigid contact path requesting only basic private contact-query
details when no rigid AVBD contact config component is present. Public
`World::collide()` and private AVBD contact snapshots still request the full
shape-index/local-point payload. The same slice caches per-contact
velocity/transform component pointers, inverse effective mass, and unit normal
impulse velocity deltas across the sequential normal solve, avoiding repeated
registry lookups and normal impulse cross/matrix work inside the inner
iterations. The audit found the source-shaped Dynamic Friction max-friction-0 row
still exercises the public World sequential-impulse contact path; the benchmark
helper does not attach private `RigidAvbdContactConfig`, and zero friction still
requires collision query, normal impulses, and penetration correction. That
slice passed focused target builds/tests, focused `/0` benchmark path checks,
`pixi run lint`, `pixi run build`, full `test_world`, `git diff --check`, and
`pixi run -e cuda test-all` on the visible NVIDIA RTX 5000 Ada host.

Previous local checkpoint: commit `7a226db4050` records the
`buildAvbdRigidContactManifoldRows()` empty-friction-descriptor early return.
That slice clears/syncs an empty friction descriptor span and returns
immediately after building normal rows when every active contact has zero
Coulomb friction-force limit, avoiding previous-friction-direction lookup/sort
and tangent-row construction for frictionless AVBD manifolds.

Current resumed slice: the default sequential-impulse rigid contact stage now
computes the combined Coulomb coefficient before tangent setup and skips
tangent-basis/effective-mass construction plus friction solve calls when the
coefficient is zero. A text `WorldStepProfile` on the source-shaped Dynamic
Friction setup showed `rigid_body_contact` dominated the frictionless step, and
the benchmark helper does not directly attach the private
`RigidAvbdContactConfig`, so this bounded slice targets the default contact
stage fixed overhead in the open max-friction-0 sweep row.

Validation for this slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_world bm_avbd_rigid_fixed_joint`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere' --gtest_brief=1`
  passed, 3 tests.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter=BM_AvbdDemo2dFrictionCoefficientSweep --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-friction-coefficient-sweep-current.json --benchmark_out_format=json'`
  passed. It recorded median CPU step times of 11.91 us, 15.47 us, 39.41 us,
  32.01 us, and 24.46 us for max friction 0, 0.5, 1.0, 2.5, and 5.0
  respectively, under high host load average `32.19, 29.17, 20.43`.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdDemo2dFrictionCoefficientSweep/0$" --benchmark_min_time=1.0s --benchmark_repetitions=5 --benchmark_out=/tmp/avbd-frictionless-sweep-row-after-default-frictionless-skip.json --benchmark_out_format=json'`
  passed. It recorded a 16.42 us median CPU step under high host load average
  `32.33, 29.42, 20.74`.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_brief=1`
  passed, 314 tests.
- `pixi run lint` passed.
- `pixi run build` passed.
- `git diff --check` passed.
- `pixi run -- build/default/cpp/Release/bin/test_world --gtest_filter='World.RigidBodyContactZeroFrictionPreservesSlidingVelocity:World.RigidBodyContactFrictionDeceleratesSlidingBody:World.RigidBodyContactFrictionRollsSlidingSphere' --gtest_brief=1`
  passed again after lint/build, 3 tests.

Evidence caveat: this is a local code-path optimization with noisy
benchmark-only reruns. It does not regenerate the tracked
friction-coefficient packet because same-source native timing and visual
capture artifacts were not rerun, and the high-load local timings are not
CPU-win evidence. The frictionless max-friction-0 CPU gap, all-coefficient
CPU-win gate, GPU parity, source-demo parity, and paper-number gates remain
open. The next preferred local gap remains a cleaner frictionless source-row
performance audit/optimization under lower host load, or GPU parity preparation
if the CPU path has no safe bounded next cut.

## Fresh Codex Goal Prompt

Copy/paste this as the first user prompt for a new Codex session:

```text
Resume PLAN-104 AVBD solver work in /home/js/dev/dartsim/dart/task_3.

Goal: continue from the single consolidated non-main branch
avbd/source-row-extraction-precheck. This branch contains the ongoing AVBD work,
PR #2977's local source-row performance work, the latest friction-sweep evidence,
the branch-consolidation handoff docs, and a merge of the latest origin/main as
of the handoff. Do not resume from deleted local branch names, remote archival
branches, or stashes unless I explicitly ask for forensic recovery.

Start by reading:
- AGENTS.md
- docs/ai/principles.md
- docs/dev_tasks/avbd_solver/README.md
- docs/dev_tasks/avbd_solver/RESUME.md
- docs/plans/104-vertex-block-descent-solver.md
- docs/plans/104-vertex-block-descent-solver/avbd-demo-corpus.md

Then verify:
- git status --short --branch
- git branch -vv
- git log --oneline --decorate -5
- git stash list
- git fetch origin, then merge origin/main before any push

Current north star: advance PLAN-104 AVBD toward source-shaped articulated rigid
and deformable row coverage with real evidence against the native source corpus.
Keep claims narrow. Do not claim a source-demo CPU win, GPU parity,
same-hardware paper-number match, broad fracture/breakable-wall coverage, or
all-coefficient friction win unless tracked artifacts directly prove it.

Current evidence: the zero-limit Coulomb contact-friction row skip,
empty-friction-descriptor early return, default contact-stage zero-friction
tangent skip, default contact-stage normal-only frictionless inner loop, basic
private contact queries for ordinary rigid solves, and cached normal-impulse
inner-loop data are landed or pending on this branch. The refreshed
friction-coefficient packet still records DART faster than the native source
runner for max friction 0.5, 1.0, 2.5, and 5.0, but slower for the frictionless
max friction 0 case. The latest local benchmark reruns after the
basic-contact-query and cached-normal-impulse slice are path validation only
because same-source native timing and visual captures were not rerun, so the
frictionless CPU gap and GPU parity remain open.

Preferred next bounded work: choose one evidence-backed PLAN-104 gap, ideally
a higher-level frictionless max-friction-0 CPU optimization/audit or GPU parity
preparation, and make a surgical change with focused validation. If the best
next step is only an audit, keep it read-only and update the handoff docs with
the evidence.

Rules:
- Use pixi run tasks.
- Run pixi run lint before every commit.
- Run build/tests appropriate to touched code.
- Before every push, merge latest origin/main into the branch.
- Do not mutate PRs, rerun hosted CI, delete remote branches, or drop stashes
  without explicit user approval.
- Preserve the old remote checkpoint refs as archival; do not re-expand them
  into local working branches for normal resume.
```

Current slice: skip Coulomb tangent-row inventory creation when every active
contact in a rigid contact manifold has a zero friction-force limit, and return
early from the empty-friction descriptor path after normal rows are built. This
keeps normal rows alive, removes useless zero-limit friction rows for
frictionless manifolds, avoids stale warm-start lookup/sort work for that empty
path, and preserves the existing descriptor layout whenever any contact in the
manifold can carry positive friction.

Current branch-consolidation snapshot:

- Branch: `avbd/source-row-extraction-precheck`.
- Upstream: `origin/avbd/source-row-extraction-precheck`.
- This is now the single non-main local branch to use for fresh-session AVBD
  resumption.
- Zero-limit friction-row implementation commit:
  `8b576ce2174 Skip zero-limit AVBD contact friction rows`.
- Branch-consolidation handoff docs are committed after
  `2f15556ef8b Record AVBD zero-limit friction handoff push`; current
  local/upstream HEAD should be the latest pushed commit on
  `avbd/source-row-extraction-precheck`.
- Latest `origin/main` was merged into this branch before committing the fresh
  Codex goal prompt. Refresh and merge `origin/main` again before any future
  push.
- Current branch had no associated GitHub PR in the latest read-only
  `gh pr list --head "$(git branch --show-current)"` snapshot.
- The zero-limit friction-row skip slice was pushed to
  `origin/avbd/source-row-extraction-precheck` via one-off HTTPS Git because
  SSH to GitHub was unavailable on this host. No hosted CI rerun, PR mutation,
  stash operation, branch cleanup, or hosted merge was performed for this
  resumed checkpoint. External mutations still require explicit maintainer/user
  approval.
- Local branch cleanup performed for consolidation:
  `avbd/source-row-perf-slice` was deleted locally after confirming its head
  `5297462d34b` is already contained in `avbd/source-row-extraction-precheck`;
  `feature/avbd-articulated-masked-rows` was deleted locally as a superseded
  raw checkpoint branch whose remote ref remains on origin; and unrelated local
  `feature/free-joint-energy-benchmarks` was deleted locally while leaving its
  remote ref intact. Do not re-expand these local branch names for routine
  resume work.

Current local validation for the zero-limit friction-row skip slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- `pixi run -- build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.RigidContactManifoldBuilderSkipsZeroLimitFrictionRows:AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows:AvbdRigidBlock.RigidContactManifoldFrictionProjectsWarmStartedDualAcrossTangentBasis'`
  passed, 3 tests.
- `pixi run -- cmake --build build/default/cpp/Release --target bm_avbd_rigid_fixed_joint`
  passed.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter=BM_AvbdDemo2dFrictionCoefficientSweep --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-friction-coefficient-sweep-after-zero-friction-skip.json --benchmark_out_format=json'`
  passed. Benchmark stdout reported high host load average
  `79.94, 114.90, 61.83`.
- `pixi run -- python scripts/write_avbd_friction_coefficient_sweep_packet.py`
  with the new benchmark JSON, five existing same-source native reference
  timing files, five existing capture manifests, and the rendered plot passed
  and regenerated the tracked packet.
- `pixi run -- python scripts/write_avbd_friction_coefficient_sweep_plot.py`
  passed and regenerated the tracked plot.
- `pixi run lint` passed.
- `pixi run -- cmake --build build/default/cpp/Release --parallel 5 --target test_avbd_rigid_block bm_avbd_rigid_fixed_joint`
  passed after lint.
- `pixi run -- build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.RigidContactManifoldBuilderSkipsZeroLimitFrictionRows:AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows:AvbdRigidBlock.RigidContactManifoldFrictionProjectsWarmStartedDualAcrossTangentBasis'`
  passed again after lint, 3 tests.
- `pixi run build` passed with `DART_PARALLEL_JOBS=5`,
  `CTEST_PARALLEL_LEVEL=5`, and `CMAKE_BUILD_PARALLEL_LEVEL=5`.
- `git diff --check` passed.

Current friction-coefficient sweep evidence after the zero-limit skip:

| Max friction | DART median CPU step | Native reference CPU step | Current result |
| ------------ | -------------------- | ------------------------- | -------------- |
| `0`          | `6.20 us`            | `2.74 us`                 | DART slower    |
| `0.5`        | `6.79 us`            | `22.04 us`                | DART faster    |
| `1.0`        | `14.87 us`           | `17.77 us`                | DART faster    |
| `2.5`        | `8.85 us`            | `20.66 us`                | DART faster    |
| `5.0`        | `7.00 us`            | `19.63 us`                | DART faster    |

Current local branch inventory:

| Branch                                | Upstream                                     | Local head at handoff | State and handling                                                                                             |
| ------------------------------------- | -------------------------------------------- | --------------------- | -------------------------------------------------------------------------------------------------------------- |
| `avbd/source-row-extraction-precheck` | `origin/avbd/source-row-extraction-precheck` | latest pushed HEAD    | Current consolidated continuation branch. It contains PR #2977's local work and all current AVBD handoff docs. |
| `main`                                | `origin/main`                                | `bb851f45360`         | Keep as base branch. Refresh before using it as a base.                                                        |

Retired local branch names now preserved only by remote refs or docs:

| Retired local branch                   | Remote ref                                    | Handling                                                                                                                                                               |
| -------------------------------------- | --------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `avbd/source-row-perf-slice`           | `origin/avbd/source-row-perf-slice`           | Local branch deleted because `5297462d34b` is already contained in the consolidated branch. PR #2977 may still exist remotely; do not resume locally from this branch. |
| `feature/avbd-articulated-masked-rows` | `origin/feature/avbd-articulated-masked-rows` | Local branch deleted as a superseded raw 33-hour checkpoint. Treat the remote as archive only unless a human explicitly asks for forensic recovery.                    |
| `feature/free-joint-energy-benchmarks` | `origin/feature/free-joint-energy-benchmarks` | Local branch deleted as unrelated to AVBD session resume. Remote ref remains untouched.                                                                                |

Current local stash inventory. Do not apply or drop these by default:

- `stash@{0}` on `avbd/source-row-extraction-precheck`:
  `codex-pr2977-switch-preserve-extraction-precheck-wip`.
- `stash@{1}` on `avbd/source-row-extraction-precheck`:
  `codex-avbd-normalize-fastpath-wip`.
- `stash@{2}` through `stash@{7}` are older
  `feature/avbd-articulated-masked-rows` pre-main-merge recovery stashes.

Local-only commits above `origin/avbd/source-row-extraction-precheck`: none
after the final handoff-doc push.

Next preferred local gaps: the frictionless max-friction-0 case and GPU parity
remain open for the friction coefficient comparison. Broader PLAN-104 gaps also
remain: source-demo CPU wins for slower rows, broader visual
breakable-wall/fracture corpus coverage, rigid contact persistence
completeness, and source-demo CPU/GPU parity. Do not count this zero-limit
friction-row skip as full paper/source-demo completion.

Fresh-session recovery commands:

```bash
git switch avbd/source-row-extraction-precheck
git status --short --branch
git log --oneline --decorate origin/avbd/source-row-extraction-precheck..HEAD
git branch -vv
git stash list
```

## Latest Resumed Checkpoint (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Keep
paper/source-demo, CPU/GPU, and paper-number claims narrow unless the artifacts
directly prove them.

Current resumed slice: the paper/source-corpus friction coefficient comparison
now has same-source reference timing and per-coefficient visual-capture
evidence. `BM_AvbdDemo2dFrictionCoefficientSweep` reuses the source-shaped
`avbd-demo2d` Dynamic Friction scene and sweeps maximum dynamic-box Coulomb
friction values 0, 0.5, 1, 2.5, and 5 over the 11-box setup.
`scripts/run_avbd_demo2d_reference_timing.py` now accepts
`--dynamic-friction-max-friction` for `--scene dynamic_friction`, so the native
source runner can time the same coefficient ladder. The py-demo scene also
reads `DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION`, and
`scripts/capture_py_demo.py` can now record caller-supplied env/metadata in
`manifest.json`, so each coefficient has a traceable visual capture. The
tracked `avbd-friction-coefficient-sweep-packet.json` validates the real DART
benchmark rows, five same-source timing rows, and five visual capture
manifests, and
`avbd-friction-coefficient-sweep-plot.svg` plots both DART and native CPU step
time against maximum friction. On this host DART is faster at max friction 0.5
and 5.0, but slower at 0, 1.0, and 2.5. This closes only the source/reference
timing-evidence and per-coefficient visual-capture gaps for the coefficient
sweep; it is not a full-coefficient CPU-win, GPU parity, or paper-number claim.

Validation for this slice:

- `pixi run -- pytest python/tests/unit/test_write_avbd_friction_coefficient_sweep_packet.py -q`
  passed, 11 tests, before generating the tracked reference packet.
- `pixi run -- cmake --build build/default/cpp/Release --target bm_avbd_rigid_fixed_joint`
  passed.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter=BM_AvbdDemo2dFrictionCoefficientSweep --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-friction-coefficient-sweep.json --benchmark_out_format=json'`
  passed and produced all five sweep values.
- Five `pixi run -- python scripts/run_avbd_demo2d_reference_timing.py --source-dir /tmp/avbd-demo2d-74699a11f858-codex --scene dynamic_friction --dynamic-friction-max-friction <value> --output /tmp/avbd-demo2d-dynamic-friction-reference-sweep-<value>.json`
  runs passed for `<value>` 0, 0.5, 1, 2.5, and 5 after checking out native
  source revision `74699a11f858`.
- `pixi run -- python scripts/write_avbd_friction_coefficient_sweep_packet.py --benchmark-json /tmp/avbd-friction-coefficient-sweep.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-0.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-0_5.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-1.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-2_5.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-5.json`
  passed and wrote the tracked packet.
- `pixi run -- python scripts/write_avbd_friction_coefficient_sweep_plot.py`
  passed and wrote the rendered DART/native plot.
- `pixi run -- python scripts/write_avbd_friction_coefficient_sweep_packet.py --benchmark-json /tmp/avbd-friction-coefficient-sweep.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-0.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-0_5.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-1.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-2_5.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-5.json --plot-svg docs/plans/104-vertex-block-descent-solver/avbd-friction-coefficient-sweep-plot.svg`
  passed and relinked the rendered plot into the tracked packet.
- `pixi run -- pytest python/tests/unit/test_capture_py_demo.py python/tests/unit/test_write_avbd_friction_coefficient_sweep_packet.py -q`
  passed, 22 tests.
- `pixi run -- bash -lc 'PYTHONPATH=build/default/cpp/Release/python:python pytest python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_dynamic_friction_scene_matches_source_row python/tests/integration/test_demos_cycle.py::test_avbd_demo2d_dynamic_friction_scene_max_friction_env -q'`
  passed, 2 tests.
- Five `LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe python scripts/capture_py_demo.py --scene avbd_demo2d_dynamic_friction --frames 24 --width 640 --height 360 --env DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION=<value> --metadata max_friction=<value> --output-dir /tmp/avbd-friction-coefficient-capture-<value>`
  captures passed for `<value>` 0, 0.5, 1, 2.5, and 5.
- `pixi run -- python scripts/write_avbd_friction_coefficient_sweep_packet.py --benchmark-json /tmp/avbd-friction-coefficient-sweep.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-0.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-0_5.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-1.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-2_5.json --reference-timing-json /tmp/avbd-demo2d-dynamic-friction-reference-sweep-5.json --capture-manifest /tmp/avbd-friction-coefficient-capture-0/manifest.json --capture-manifest /tmp/avbd-friction-coefficient-capture-0_5/manifest.json --capture-manifest /tmp/avbd-friction-coefficient-capture-1/manifest.json --capture-manifest /tmp/avbd-friction-coefficient-capture-2_5/manifest.json --capture-manifest /tmp/avbd-friction-coefficient-capture-5/manifest.json --plot-svg docs/plans/104-vertex-block-descent-solver/avbd-friction-coefficient-sweep-plot.svg`
  passed and embedded the visual sweep hashes in the tracked packet.
- `pixi run -- pytest python/tests/unit/test_write_avbd_friction_coefficient_sweep_packet.py python/tests/unit/test_benchmark_display_names.py python/tests/unit/test_run_performance_dashboard_benchmarks.py -q`
  passed, 20 tests, after lint.
- `pixi run lint` passed.
- `pixi run build` passed.
- `git diff --check` passed.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. After this slice is committed,
use the latest local HEAD as the resume point. Do not push, rerun hosted CI,
mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gaps: CPU optimization for the slower friction
coefficients and GPU parity remain open for the friction coefficient
comparison. Broader PLAN-104 gaps also remain: source-demo CPU wins for the
slower rows, broader visual breakable-wall/fracture corpus coverage, rigid
contact persistence completeness, and source-demo CPU/GPU parity. Do not count
this packet as full paper/source-demo completion.

## Previous Explicit Hand-Off Stop (2026-06-12)

User directive: stop working further, only ensure the hand-off docs, and then
literally stop. Do not continue implementation, verification, hosted CI work,
branch cleanup, PR mutation, push, merge, review-comment handling, or stash
cleanup from this checkpoint unless a future user request explicitly asks for
that specific action. This section is a hand-off snapshot only and supersedes
older stop-handoff snapshots lower in this file.

North star if work is explicitly resumed later: continue PLAN-104 AVBD toward
source-shaped articulated rigid and deformable row coverage with evidence
against the native source corpus. Keep claims narrow. The recent breakable
joint/motor packets are benchmark-only scale evidence for existing public
fixed/spherical joint and revolute/prismatic motor row families; they are not a
broad motor lifecycle corpus, broad visual breakable-wall/fracture corpus,
CPU-win, GPU parity, source-demo parity, or same-hardware paper-number claim.

Current checkout snapshot for this docs-only hand-off:

- Branch: `avbd/source-row-extraction-precheck`.
- Upstream: `origin/avbd/source-row-extraction-precheck`.
- Local HEAD: `0952636627d Add AVBD breakable motor scale packet`.
- Status before this docs edit: clean, ahead of upstream by 21 commits.
- Current branch has no associated GitHub PR.
- No fresh lint/build/test/CI, `git diff --check`, hosted rerun, PR refresh
  beyond a read-only `gh pr status`, push, branch deletion, merge, or stash
  operation was performed for this hand-off. The user explicitly asked to stop
  after the hand-off docs.

Local-only commits above `origin/avbd/source-row-extraction-precheck`, newest
first:

- `0952636627d Add AVBD breakable motor scale packet`
- `95251eec1c3 Add AVBD breakable joint scale packet`
- `1c2059c4c08 Add AVBD current-pose tiny motor persistence coverage`
- `8c59eb9f203 Record AVBD final stop handoff`
- `ba56bd46517 Add AVBD high-ratio sweep stability plot`
- `87a7825b1dc Add AVBD high-ratio iteration sweep packet`
- `702e6021676 Record AVBD final handoff state`
- `8e12c559938 Add AVBD high-ratio iteration sweep benchmark`
- `3732c21233d Record AVBD current-state gap audit`
- `30c7e8f6239 Add AVBD direct fixed movable reset coverage`
- `33247979e31 Add AVBD direct spherical movable reset coverage`
- `3aa6d5ab3f4 Add AVBD direct prismatic movable reset coverage`
- `42088402003 Record AVBD critical handoff state`
- `456b931a57b Broaden AVBD articulated reset axis coverage`
- `6e1492826c7 Broaden AVBD private motor persistence axis coverage`
- `68af00b8817 Record AVBD stop handoff snapshot`
- `533dc490d87 Reuse AVBD rigid motor row scratch`
- `77404f63496 Avoid contact-manifold row builder heap scratch for small AVBD inputs`
- `04a369222a7 Record AVBD literal stop handoff`
- `13604d8b8b5 Reuse AVBD distance spring row scratch`
- `51eb9b48e08 Record AVBD angular motor validation evidence`

Current local branch inventory:

| Branch                                 | Upstream                                      | Local head at handoff | State and handling                                                                                                          |
| -------------------------------------- | --------------------------------------------- | --------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `avbd/source-row-extraction-precheck`  | `origin/avbd/source-row-extraction-precheck`  | `0952636627d`         | Current consolidated continuation branch; ahead of origin by 21 commits before this docs edit.                              |
| `avbd/source-row-perf-slice`           | `origin/avbd/source-row-perf-slice`           | `5297462d34b`         | PR #2977 branch; read-only status showed one failing visible check and up-to-date. Do not mutate without explicit approval. |
| `avbd/articulated-stiffness-roundtrip` | `origin/avbd/articulated-stiffness-roundtrip` | `43787619654`         | #2975-era branch reported merged by the user; cleanup still requires explicit approval.                                     |
| `feature/avbd-articulated-masked-rows` | `origin/feature/avbd-articulated-masked-rows` | `d25e5177d9c`         | Raw 33-hour safety checkpoint; keep until the split AVBD work is safely landed or explicitly retired.                       |
| `feature/free-joint-energy-benchmarks` | `origin/feature/free-joint-energy-benchmarks` | `d13c97b5f0c`         | Unrelated local branch; do not touch during AVBD handoff.                                                                   |
| `main`                                 | `origin/main`                                 | `7d05d7b9ea7`         | Local main matched the last checked origin/main snapshot; refresh before using it as a base.                                |

Current local stash inventory. Do not apply or drop these by default:

- `stash@{0}` on `avbd/source-row-extraction-precheck`:
  `codex-pr2977-switch-preserve-extraction-precheck-wip`.
- `stash@{1}` on `avbd/source-row-extraction-precheck`:
  `codex-avbd-normalize-fastpath-wip`.
- `stash@{2}` through `stash@{7}` are older
  `feature/avbd-articulated-masked-rows` pre-main-merge recovery stashes.

Read-only GitHub status snapshot for AVBD:

- Current branch: no associated PR.
- PR #2977, `avbd/source-row-perf-slice`: open, up-to-date, with one visible
  failing check at the time of the read-only snapshot. Refresh before acting.
- User-reported merged PRs during the broader AVBD split include #2967, #2968,
  #2969, #2973, and #2975. Do not mutate branches or GitHub state based only on
  this hand-off note; refresh first if the user explicitly asks to resume that
  work.

Stopped plan: no implementation should continue from this checkpoint. If a
future user explicitly asks to resume implementation, start with a fresh
read-only audit of the genuinely open PLAN-104 gaps before editing. The next
likely gaps remain broader visual breakable-wall/fracture corpus coverage,
rigid contact persistence completeness, and source-demo CPU/GPU parity. Do not
spend more work on source-row overhead cleanup unless a new audit identifies a
specific blocker.

Fresh-session recovery commands:

```bash
git switch avbd/source-row-extraction-precheck
git status --short --branch
git log --oneline --decorate origin/avbd/source-row-extraction-precheck..HEAD
git branch -vv
git stash list
```

## Latest Resumed Checkpoint (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count the new breakable motor scale packet as a broad motor lifecycle corpus,
breakable-wall/fracture corpus, CPU-win, GPU, paper-number, or source-demo
parity gate without visual corpus, reference comparison, GPU, and
same-hardware evidence.

Current resumed slice: the public articulated breakable motor benchmark
surface now has a validated benchmark-only scale packet,
`avbd-breakable-motor-scale-packet.json`, generated from a real
`BM_AvbdArticulatedBreakableMotorStep`,
`BM_AvbdArticulatedPrismaticBreakableMotorStep`,
`BM_AvbdArticulatedWorldPrismaticBreakableMotorStep`, and
`BM_AvbdArticulatedWorldRevoluteBreakableMotorStep` run over 1, 8, and 32
motors. The packet validates finite timing rows and exact `motors` plus
`breakable_motors` counters for the same-multibody/world-link
revolute/prismatic public motor row families.

Validation for this slice:

- `pixi run -- pytest python/tests/unit/test_write_avbd_breakable_motor_scale_packet.py -q`
  passed, 6 tests.
- `pixi run -- cmake --build build/default/cpp/Release --target bm_avbd_rigid_fixed_joint`
  passed, no work to do.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_AvbdArticulated(BreakableMotor|PrismaticBreakableMotor|WorldPrismaticBreakableMotor|WorldRevoluteBreakableMotor)Step/(1|8|32)$" --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-breakable-motor-scale.json --benchmark_out_format=json'`
  passed and produced all four articulated breakable motor row families over
  1/8/32.
- `pixi run -- python scripts/write_avbd_breakable_motor_scale_packet.py --benchmark-json /tmp/avbd-breakable-motor-scale.json`
  passed and wrote the tracked packet.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. After this slice is committed,
use the latest local HEAD as the resume point. Do not push, rerun hosted CI,
mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gap after this slice remains broader visual
breakable-wall/fracture corpus coverage, rigid contact persistence
completeness, and source-demo CPU/GPU parity. Keep all claims narrow unless
corpus or benchmark evidence supports them.

## Previous Breakable Joint Scale Checkpoint (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count the breakable joint scale packet as a breakable-wall/fracture corpus,
CPU-win, GPU, paper-number, or source-demo parity gate without visual corpus,
reference comparison, GPU, and same-hardware evidence.

Current resumed slice: the public fixed/spherical breakable point-joint
benchmark surface now has a validated benchmark-only scale packet,
`avbd-breakable-joint-scale-packet.json`, generated from a real
`BM_AvbdRigidBreakableJointStep`,
`BM_AvbdRigidSphericalBreakableJointStep`,
`BM_AvbdArticulatedBreakableJointStep`,
`BM_AvbdArticulatedWorldSphericalBreakableJointStep`, and
`BM_AvbdArticulatedSphericalPairBreakableJointStep` run over 1, 8, and 32
breakable joints. The packet validates finite timing rows and exact
`breakable_joints` counters for each fixed/spherical public row family.

Validation for this slice:

- `pixi run -- pytest python/tests/unit/test_write_avbd_breakable_joint_scale_packet.py -q`
  passed, 5 tests.
- `pixi run -- cmake --build build/default/cpp/Release --target bm_avbd_rigid_fixed_joint`
  passed.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter="BM_Avbd(Rigid(BreakableJoint|SphericalBreakableJoint)|Articulated(BreakableJoint|WorldSphericalBreakableJoint|SphericalPairBreakableJoint))Step/(1|8|32)$" --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-breakable-joint-scale.json --benchmark_out_format=json'`
  passed and produced all five fixed/spherical row families over 1/8/32.
- `pixi run -- python scripts/write_avbd_breakable_joint_scale_packet.py --benchmark-json /tmp/avbd-breakable-joint-scale.json`
  passed and wrote the tracked packet.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. After this slice is committed,
use the latest local HEAD as the resume point. Do not push, rerun hosted CI,
mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gap after this slice remains broader visual
breakable-wall/fracture corpus coverage, breakable motor scale evidence, rigid
contact persistence completeness, and source-demo CPU/GPU parity. Keep all
claims narrow unless corpus or benchmark evidence supports them.

## Previous Current-Pose Motor Persistence Checkpoint (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count focused articulated motor persistence tests as fracture-corpus, CPU-win,
GPU, paper-number, or full parameter-sweep gates without corpus, benchmark
JSON, plot data, and same-hardware comparisons.

Current resumed slice: current-pose private movable-pair revolute and
prismatic AVBD motor rows now have focused simulation-mode binary save/load
regressions proving tiny restored effort limits remain effective after reload.
The tests preserve the generated private `AvbdRigidWorldPointJointConfig`,
command, effort-limit, mask, and non-cardinal basis state, then step the
restored world to verify the hard anchor/hinge/masked rows stay active while
the free-axis motor does not become an unbounded velocity target.

Validation for this slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_variational_integration`
  passed.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_variational_integration --gtest_filter='VariationalIntegration.AvbdRevolutePointJointCurrentPoseTinyTorqueLimitSurvivesSaveLoad:VariationalIntegration.AvbdPrismaticPointJointCurrentPoseTinyForceLimitSurvivesSaveLoad' --gtest_brief=1"`
  passed, 2 tests.
- `pixi run lint` passed.
- Re-running the focused target/test filter after lint passed.
- `pixi run build` passed.
- `git diff --check` passed.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. After this slice is committed,
use the latest local HEAD as the resume point. Do not push, rerun hosted CI,
mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gap after this slice remains broader articulated/fracture
corpus coverage, then rigid contact persistence completeness and source-demo
CPU/GPU parity. Keep all claims narrow unless corpus or benchmark evidence
supports them.

## Previous High-Ratio Sweep Checkpoint (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count focused benchmark rows as CPU-win, GPU, paper-number, or full parameter
sweep gates without benchmark JSON, plot data, and same-hardware comparisons.

Current resumed slice: the paper-scale high-ratio articulated-chain
iteration-count sweep now records finite replay stability counters and has a
tracked benchmark/stability-plot packet,
`avbd-paper-scale-high-ratio-iteration-sweep-packet.json`, plus a rendered SVG
plot, `avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`, generated from a
real `BM_AvbdPaperScaleHighRatioChainIterationSweep` run over 25, 50, 100, and
200 max-iteration budgets. The packet validates the existing
50-link/50,000:1 fixture, replay-reset path, tolerance, and metadata counters,
requires finite 32-step replay for each budget, and records sorted
timing/stability plot data. This does not provide a same-hardware paper-number
comparison, broad articulated stability proof, or GPU parity claim.

Validation for this slice:

- `pixi run -- pytest python/tests/unit/test_write_avbd_paper_scale_high_ratio_iteration_sweep_packet.py python/tests/unit/test_write_avbd_paper_scale_high_ratio_iteration_sweep_plot.py -q`
  passed, 7 tests.
- `pixi run -- cmake --build build/default/cpp/Release --target bm_avbd_rigid_fixed_joint`
  passed.
- `pixi run -- bash -lc 'build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint --benchmark_filter=BM_AvbdPaperScaleHighRatioChainIterationSweep --benchmark_min_time=0.5s --benchmark_repetitions=3 --benchmark_out=/tmp/avbd-paper-scale-high-ratio-iteration-sweep.json --benchmark_out_format=json'`
  passed and produced all four budget rows with finite replay counters.
- `pixi run -- python scripts/write_avbd_paper_scale_high_ratio_iteration_sweep_packet.py --benchmark-json /tmp/avbd-paper-scale-high-ratio-iteration-sweep.json && pixi run -- python scripts/write_avbd_paper_scale_high_ratio_iteration_sweep_plot.py && pixi run -- python scripts/write_avbd_paper_scale_high_ratio_iteration_sweep_packet.py --benchmark-json /tmp/avbd-paper-scale-high-ratio-iteration-sweep.json --plot-svg docs/plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`
  passed and wrote the tracked packet plus rendered plot.
- `pixi run lint` passed.
- `pixi run build` passed.
- `git diff --check` passed.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. After this slice is committed,
use the latest local HEAD as the resume point. Do not push, rerun hosted CI,
mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gap after this slice is broader articulated/fracture corpus
coverage, then rigid contact persistence completeness and source-demo CPU/GPU
parity. Keep all claims narrow unless corpus or benchmark evidence supports
them.

## Final Stop Handoff (2026-06-12)

User directive: stop working further, only ensure the hand-off docs, and then
literally stop. Do not continue implementation, verification, hosted CI work,
branch cleanup, PR mutation, push, merge, or review-comment handling from this
checkpoint unless a future user request explicitly asks for that specific
action. This section is a hand-off snapshot only.

North star when work is explicitly resumed: continue PLAN-104 AVBD toward
source-shaped articulated rigid and deformable row coverage with evidence
against the native source corpus. Do not count source-row overhead cleanup or
focused articulated lifecycle tests as CPU-win, GPU, or paper-number gates;
those gates require dedicated corpus and benchmark evidence.

Current checkout snapshot before this docs-only handoff edit:

- Branch: `avbd/source-row-extraction-precheck`.
- Upstream: `origin/avbd/source-row-extraction-precheck`.
- Local HEAD:
  `ba56bd46517 Add AVBD high-ratio sweep stability plot`.
- Status before this docs edit: clean, ahead of upstream by 17 commits.
- The `BM_AvbdPaperScaleHighRatioChainIterationSweep` benchmark row exists and
  is wired into the display-name/dashboard-runner surfaces. The branch now has
  a tracked benchmark/stability packet,
  `docs/plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-packet.json`,
  and a rendered SVG plot,
  `docs/plans/104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`,
  generated from a real four-budget sweep over the 50-link/50,000:1 fixture.
  Do not treat that as a same-hardware paper-number comparison, broad
  articulated stability proof, or GPU parity claim.
- This stop-only handoff intentionally ran no lint/build/test/CI,
  `git diff --check`, PR refresh, or hosted rerun. The user explicitly asked
  for no further verification and to stop after the hand-off docs.
- No push, PR mutation, hosted CI rerun, merge, branch deletion, or stash
  operation was performed for this handoff.

Local-only commits above `origin/avbd/source-row-extraction-precheck`, newest
first:

- `ba56bd46517 Add AVBD high-ratio sweep stability plot`
- `87a7825b1dc Add AVBD high-ratio iteration sweep packet`
- `702e6021676 Record AVBD final handoff state`
- `8e12c559938 Add AVBD high-ratio iteration sweep benchmark`
- `3732c21233d Record AVBD current-state gap audit`
- `30c7e8f6239 Add AVBD direct fixed movable reset coverage`
- `33247979e31 Add AVBD direct spherical movable reset coverage`
- `3aa6d5ab3f4 Add AVBD direct prismatic movable reset coverage`
- `42088402003 Record AVBD critical handoff state`
- `456b931a57b Broaden AVBD articulated reset axis coverage`
- `6e1492826c7 Broaden AVBD private motor persistence axis coverage`
- `68af00b8817 Record AVBD stop handoff snapshot`
- `533dc490d87 Reuse AVBD rigid motor row scratch`
- `77404f63496 Avoid contact-manifold row builder heap scratch for small AVBD inputs`
- `04a369222a7 Record AVBD literal stop handoff`
- `13604d8b8b5 Reuse AVBD distance spring row scratch`
- `51eb9b48e08 Record AVBD angular motor validation evidence`

Current local branch inventory:

| Branch                                 | Upstream                                      | Local head at handoff | State and handling                                                                                    |
| -------------------------------------- | --------------------------------------------- | --------------------- | ----------------------------------------------------------------------------------------------------- |
| `avbd/source-row-extraction-precheck`  | `origin/avbd/source-row-extraction-precheck`  | `ba56bd46517`         | Current consolidated continuation branch; ahead of origin by 17 commits before this docs edit.        |
| `avbd/source-row-perf-slice`           | `origin/avbd/source-row-perf-slice`           | `5297462d34b`         | PR #2977 branch; do not mutate or rerun CI without explicit approval and a fresh PR status refresh.   |
| `avbd/articulated-stiffness-roundtrip` | `origin/avbd/articulated-stiffness-roundtrip` | `43787619654`         | #2975-era branch reported merged by the user; cleanup still requires explicit approval.               |
| `feature/avbd-articulated-masked-rows` | `origin/feature/avbd-articulated-masked-rows` | `d25e5177d9c`         | Raw 33-hour safety checkpoint; keep until the split AVBD work is safely landed or explicitly retired. |
| `feature/free-joint-energy-benchmarks` | `origin/feature/free-joint-energy-benchmarks` | `d13c97b5f0c`         | Unrelated local branch; do not touch during AVBD handoff.                                             |
| `main`                                 | `origin/main`                                 | `7d05d7b9ea7`         | Local main matched the last checked origin/main snapshot; refresh before using it as a base.          |

Current local stash inventory. Do not apply or drop these by default:

- `stash@{0}` on `avbd/source-row-extraction-precheck`:
  `codex-pr2977-switch-preserve-extraction-precheck-wip`.
- `stash@{1}` on `avbd/source-row-extraction-precheck`:
  `codex-avbd-normalize-fastpath-wip`.
- `stash@{2}` through `stash@{7}` are older
  `feature/avbd-articulated-masked-rows` pre-main-merge recovery stashes.

GitHub/PR state was not refreshed during this stop-only handoff. User-reported
merged PRs during the broader AVBD split included #2967, #2968, #2969, #2973,
and #2975. Last known PR #2977 state from earlier in the session was that it
remained open with one visible failing check, but that may be stale. A future
session should refresh PR status read-only before deciding whether to address
CI. Do not reply to bot comments or mutate review threads unless the user
explicitly approves the required GitHub action.

Stopped next-work plan: after the sweep packet/plot commit, the next planned
step was a read-only audit for a concrete broader articulated/fracture corpus
coverage gap. That audit was not started after the user's stop directive. If a
future user explicitly asks to resume implementation, start with that audit
instead of assuming a missing case.

Fresh-session recovery commands:

```bash
git switch avbd/source-row-extraction-precheck
git status --short --branch
git log --oneline --decorate origin/avbd/source-row-extraction-precheck..HEAD
git branch -vv
git stash list
```

If a future user explicitly asks to resume implementation, prefer a fresh
read-only audit before editing. The next real PLAN-104 gaps remain broader
private articulated motor/fracture lifecycle and corpus coverage, then rigid
contact persistence completeness, then source-demo CPU/GPU parity. Do not spend
more work on source-row overhead cleanup unless a new audit identifies a
specific blocker.

## Latest Current-State Audit (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count source-row overhead cleanup or focused articulated lifecycle tests as
CPU-win, GPU, or paper-number gates; those gates require dedicated corpus and
benchmark evidence.

Current resumed slice: audited the current task docs against the checked-in C++
and dartpy evidence before adding more coverage. The current tree already has
narrow breakable benchmark rows, public world-fixed reset, dartpy
fixed/spherical break/skip/reset endpoint-shape checks, and non-cardinal dartpy
same-multibody/world-link one-DOF reset checks. Those items should not be used
as fresh implementation targets unless a future audit finds a concrete missing
case.

Fresh-session state for this audit: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. The branch was clean before this
docs-only audit edit and was ahead of origin by twelve local commits. Do not
push, rerun hosted CI, mutate PRs, or clean/delete branches without explicit
user approval.

PR #2977 read-only CI refresh: the visible failure is Linux `Debug Tests`
cancelled at the job timeout during the Debug Python phase after Debug C++ tests
passed. The log reached the `dartpy` debug build/link step before cancellation
and did not show a Python assertion or test failure. Treat this as an open PR CI
status that needs maintainer-approved rerun or a fresh failure-specific audit;
no hosted CI rerun was performed from this branch.

Next preferred local gap after this audit remains broader private articulated
motor/fracture lifecycle/corpus coverage, then rigid contact persistence
completeness, then source-demo CPU/GPU evidence. More source-row overhead
cleanup should not be treated as a CPU-win, GPU, or paper-number gate.

## Latest Direct Private Movable-Pair Reset Checkpoint (2026-06-12)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count source-row overhead cleanup or focused articulated lifecycle tests as
CPU-win, GPU, or paper-number gates; those gates require dedicated corpus and
benchmark evidence.

Current resumed slice: direct private same-multibody fixed, prismatic velocity,
and spherical point-joint configs now have the same break/skip/reset coverage
shape as the neighboring direct private movable-pair revolute reset path. The
new C++ regressions use explicit off-origin anchors, cover all-axis fixed rows,
a non-cardinal prismatic slider basis, and spherical linear-only rows, apply
opposing endpoint forces while broken rows are skipped, then clear `broken` and
raise `breakForce` to verify the existing private
`AvbdRigidWorldPointJointConfig` still re-engages the expected masked hard rows
across two movable same-multibody endpoints. The prismatic case also reverses
the command and verifies the free-axis motor row samples the updated velocity.

Validation so far for this slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_variational_integration`
  passed.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_variational_integration --gtest_filter='VariationalIntegration.AvbdBreakableFixedPointJointConfigResetReengagesMovablePair:VariationalIntegration.AvbdBreakableRevoluteVelocityPointJointConfigResetReengagesMovablePair:VariationalIntegration.AvbdBreakablePrismaticVelocityPointJointConfigResetReengagesMovablePair:VariationalIntegration.AvbdBreakableSphericalPointJointConfigResetReengagesMovablePair:VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorResetReengagesMovablePair:VariationalIntegration.AvbdSphericalPointJointCurrentPoseExtractorResetReengagesMovablePair' --gtest_brief=1"`
  passed, 6 tests.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. The previous local checkpoint is
`42088402003 Record AVBD critical handoff state`; after this slice is
committed, use the latest local HEAD as the resume point. Do not push, rerun
hosted CI, mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gap after this slice remains broader PLAN-104 articulated
lifecycle/corpus coverage, then rigid contact persistence completeness, then
source-demo CPU/GPU evidence. More source-row overhead cleanup should not be
treated as a CPU-win, GPU, or paper-number gate.

## Critical Stop Handoff (2026-06-12)

User directive: stop all further work and only ensure the hand-off docs. Do
not continue implementation, verification, branch cleanup, PR mutation, hosted
CI work, push, or merge from this checkpoint unless a future user request
explicitly asks for it. This section is intentionally a hand-off snapshot, not
new validation evidence.

North star when work is explicitly resumed: continue PLAN-104 AVBD toward
source-shaped articulated rigid and deformable row coverage with evidence
against the native source corpus. Do not count source-row overhead cleanup or
focused articulated lifecycle tests as CPU-win, GPU, or paper-number gates;
those gates require dedicated corpus and benchmark evidence.

Current checkout snapshot before this docs-only handoff edit:

- Branch: `avbd/source-row-extraction-precheck`.
- Upstream: `origin/avbd/source-row-extraction-precheck`.
- Local HEAD:
  `456b931a57bfc12bbd9b7b1eea9adbe5c7ed1ca1 Broaden AVBD articulated reset axis coverage`.
- Status before this docs edit: clean, ahead of upstream by eight commits.
- Local-only commits above upstream, newest first:
  `456b931a57b Broaden AVBD articulated reset axis coverage`,
  `6e1492826c7 Broaden AVBD private motor persistence axis coverage`,
  `68af00b8817 Record AVBD stop handoff snapshot`,
  `533dc490d87 Reuse AVBD rigid motor row scratch`,
  `77404f63496 Avoid contact-manifold row builder heap scratch for small AVBD inputs`,
  `04a369222a7 Record AVBD literal stop handoff`,
  `13604d8b8b5 Reuse AVBD distance spring row scratch`, and
  `51eb9b48e08 Record AVBD angular motor validation evidence`.
- This docs-only handoff intentionally has no new build/test/CI or
  `git diff --check` verification. The user explicitly asked to stop without
  further verification; only the repository-required pre-commit
  `pixi run lint` is expected if this handoff is committed locally. Do not
  infer that this handoff edit was pushed unless a later session records that
  explicitly.

Current local branch inventory:

| Branch                                 | Upstream                                      | Local head at handoff | State and handling                                                                                    |
| -------------------------------------- | --------------------------------------------- | --------------------- | ----------------------------------------------------------------------------------------------------- |
| `avbd/source-row-extraction-precheck`  | `origin/avbd/source-row-extraction-precheck`  | `456b931a57b`         | Current consolidated continuation branch; ahead of origin by eight commits before this docs edit.     |
| `avbd/source-row-perf-slice`           | `origin/avbd/source-row-perf-slice`           | `5297462d34b`         | PR #2977 branch; do not mutate or rerun CI without explicit approval and a fresh PR status refresh.   |
| `avbd/articulated-stiffness-roundtrip` | `origin/avbd/articulated-stiffness-roundtrip` | `43787619654`         | #2975-era branch reported merged by the user; cleanup still requires explicit approval.               |
| `feature/avbd-articulated-masked-rows` | `origin/feature/avbd-articulated-masked-rows` | `d25e5177d9c`         | Raw 33-hour safety checkpoint; keep until the split AVBD work is safely landed or explicitly retired. |
| `feature/free-joint-energy-benchmarks` | `origin/feature/free-joint-energy-benchmarks` | `d13c97b5f0c`         | Unrelated local branch; do not touch during AVBD handoff.                                             |
| `main`                                 | `origin/main`                                 | `7d05d7b9ea7`         | Local main matched the last checked origin/main snapshot; refresh before using it as a base.          |

Current local stash inventory. Do not apply or drop these by default:

- `stash@{0}` on `avbd/source-row-extraction-precheck`:
  `codex-pr2977-switch-preserve-extraction-precheck-wip`.
- `stash@{1}` on `avbd/source-row-extraction-precheck`:
  `codex-avbd-normalize-fastpath-wip`.
- `stash@{2}` through `stash@{7}` are older
  `feature/avbd-articulated-masked-rows` pre-main-merge recovery stashes.

PR #2977 state was not refreshed during this stop-only handoff. The last known
details below may be stale; a future session should refresh them read-only
before making any PR decision. Do not reply to bot comments or mutate review
threads.

Fresh-session recovery command:

```bash
git switch avbd/source-row-extraction-precheck
git status --short --branch
git log --oneline --decorate -12
git stash list
```

If a future user explicitly asks to resume implementation, the next preferred
local gap is broader articulated lifecycle coverage rather than more
source-row overhead cleanup. The concrete unstarted direct private
same-multibody prismatic velocity point-joint reset slice identified at this
stop was completed by the latest checkpoint above. After that, prefer rigid
contact persistence completeness and then source-demo CPU/GPU corpus evidence.
Keep all claims narrow unless corpus or benchmark evidence supports them.

## Latest Same-Multibody Reset Checkpoint (2026-06-11)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count source-row overhead cleanup or focused articulated lifecycle tests as
CPU-win, GPU, or paper-number gates; those gates require dedicated corpus and
benchmark evidence.

Current resumed slice: same-multibody articulated one-DOF reset/re-engagement
coverage now uses non-cardinal axes in four reset paths: the public movable
offset revolute path, the direct private movable revolute config path, the
generated current-pose movable revolute config path, and the generated
current-pose movable prismatic config path. The affected tests now measure
hinge/slider progress against the configured arbitrary axis instead of
cardinal yaw/X-axis shortcuts.

Validation so far for this slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_variational_integration`
  passed.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_variational_integration --gtest_filter='VariationalIntegration.AvbdPublicArticulatedRevoluteBreakForceResetReengagesMovableOffsetPair:VariationalIntegration.AvbdBreakableRevoluteVelocityPointJointConfigResetReengagesMovablePair:VariationalIntegration.AvbdRevolutePointJointCurrentPoseExtractorResetReengagesMovablePair:VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorResetReengagesMovablePair' --gtest_brief=1"`
  passed, 4 tests.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. The previous local checkpoint is
`6e1492826c7 Broaden AVBD private motor persistence axis coverage`; after this
slice is committed, use the latest local HEAD as the resume point. Do not push,
rerun hosted CI, mutate PRs, or clean/delete branches without explicit user
approval.

Next preferred local gap after this slice remains broader PLAN-104 articulated
lifecycle/corpus coverage, then rigid contact persistence completeness, then
source-demo CPU/GPU evidence. More source-row overhead cleanup should not be
treated as a CPU-win, GPU, or paper-number gate.

## Previous Non-Cardinal Persistence Checkpoint (2026-06-11)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count source-row overhead cleanup or focused articulated lifecycle tests as
CPU-win, GPU, or paper-number gates; those gates require dedicated corpus and
benchmark evidence.

Current resumed slice: direct private articulated revolute/prismatic velocity
point-joint broken-state save/load/reset coverage now uses non-cardinal
free-axis bases for both child-link and parent-link world-link endpoint
polarities. The four strengthened C++ tests verify that serialized
`AvbdRigidWorldPointJointConfig` linear/angular axis bases survive while the
joint is broken, and that reset re-engages the masked hard rows plus the
free-axis motor row with the expected endpoint-polarity sign.

Validation so far for this slice:

- `pixi run -- cmake --build build/default/cpp/Release --target test_variational_integration`
  passed.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_variational_integration --gtest_filter='VariationalIntegration.AvbdBreakableRevoluteVelocityPointJointConfigSurvivesSaveLoadAndReset:VariationalIntegration.AvbdBreakableRevoluteVelocityParentPointJointConfigSurvivesSaveLoadAndReset:VariationalIntegration.AvbdBreakablePrismaticVelocityPointJointConfigSurvivesSaveLoadAndReset:VariationalIntegration.AvbdBreakablePrismaticVelocityParentPointJointConfigSurvivesSaveLoadAndReset' --gtest_brief=1"`
  passed, 4 tests.
- `pixi run build` passed.
- `pixi run lint` passed.
- `git diff --check` passed.

Fresh-session state for this slice: branch
`avbd/source-row-extraction-precheck`, upstream
`origin/avbd/source-row-extraction-precheck`. The previous local checkpoint was
`68af00b8817 Record AVBD stop handoff snapshot`; after this slice is committed,
use the latest local HEAD as the resume point. Do not push, rerun hosted CI,
mutate PRs, or clean/delete branches without explicit user approval.

Next preferred local gap after this slice remains broader PLAN-104 articulated
lifecycle/corpus coverage, then rigid contact persistence completeness, then
source-demo CPU/GPU evidence. More source-row overhead cleanup should not be
treated as a CPU-win, GPU, or paper-number gate.

## Previous Literal Stop Handoff (2026-06-11)

User directive: stop all further work. This handoff exists only so a fresh
Claude/Codex session can recover the current state; do not continue
implementation, branch cleanup, PR mutation, CI reruns, push, or verification
from this checkpoint unless a future user request explicitly asks for it.

North star when work is explicitly resumed: continue PLAN-104 AVBD toward
source-shaped articulated rigid and deformable row coverage with evidence
against the native source corpus. Do not count the source-row overhead cleanup
commits below as a CPU-win, GPU, or paper-number gate; those gates require
dedicated corpus evidence.

Current checkout snapshot before this docs-only handoff edit:

- Branch: `avbd/source-row-extraction-precheck`.
- Upstream: `origin/avbd/source-row-extraction-precheck`.
- Local HEAD: `533dc490d87 Reuse AVBD rigid motor row scratch`.
- Status before this docs edit: clean, ahead of upstream by five commits.
- Local-only commits above upstream:
  `51eb9b48e08 Record AVBD angular motor validation evidence`,
  `13604d8b8b5 Reuse AVBD distance spring row scratch`,
  `04a369222a7 Record AVBD literal stop handoff`,
  `77404f63496 Avoid contact-manifold row builder heap scratch for small AVBD inputs`,
  `533dc490d87 Reuse AVBD rigid motor row scratch`.
- This docs-only handoff intentionally has no new lint/build/test/CI or
  `git diff --check` verification. The user explicitly asked to stop without
  further verification.

Current local branch inventory:

- `avbd/source-row-extraction-precheck` - active consolidated continuation
  branch, tracking origin and ahead by five commits.
- `avbd/source-row-perf-slice` - tracks origin; PR #2977 head branch at
  `5297462d34b Restore AVBD prepare contact query warmup`.
- `avbd/articulated-stiffness-roundtrip` - tracks origin at
  `43787619654 Merge branch 'main' into avbd/articulated-stiffness-roundtrip`.
- `feature/avbd-articulated-masked-rows` - tracks origin at
  `d25e5177d9c WIP: AVBD articulated masked-row solver -- 33h checkpoint`.
- `feature/free-joint-energy-benchmarks` - tracks origin at
  `d13c97b5f0c Broaden FreeJoint general semantics coverage`.
- `main` - tracks origin at `7d05d7b9ea7 Add reduced ABD comparison packets (#2976)`.

Current local stash inventory. Do not apply or drop these by default:

- `stash@{0}` on `avbd/source-row-extraction-precheck`:
  `codex-pr2977-switch-preserve-extraction-precheck-wip`.
- `stash@{1}` on `avbd/source-row-extraction-precheck`:
  `codex-avbd-normalize-fastpath-wip`.
- `stash@{2}` through `stash@{7}` are older
  `feature/avbd-articulated-masked-rows` pre-main-merge recovery stashes.

Current PR #2977 snapshot from read-only `gh pr view`:

- PR: <https://github.com/dartsim/dart/pull/2977>
  (`Trim AVBD source-row contact prep overhead`).
- Head branch/SHA: `avbd/source-row-perf-slice` at
  `5297462d34b6118e600647cf18cdd7f13e0182b3`.
- Base: `main`; state: open; draft: false; merge state: `BLOCKED`.
- Latest visible completed checks were green/skipped/neutral, including CUDA
  Build, CodeQL C++/Python, Codecov project/patch, Linux Release Tests,
  Windows Release Tests, macOS arm64 tests, wheels, ReadTheDocs, and lint.
- One Linux `Debug Tests` check was still `IN_PROGRESS` at handoff
  (`https://github.com/dartsim/dart/actions/runs/27386716800/job/80936707478`).
  Refresh before making any PR decision.
- No PR comment, review-thread mutation, rerun, merge, or push was performed
  for this handoff.

Fresh-session recovery command:

```bash
git checkout avbd/source-row-extraction-precheck
git status -sb
git log --oneline --decorate -8
git stash list
gh pr view 2977 --repo dartsim/dart --json number,title,state,headRefName,headRefOid,baseRefName,mergeStateStatus,isDraft,statusCheckRollup
```

If a future user asks to resume implementation, prefer the plan's current
higher-priority PLAN-104 gaps over more source-row overhead cleanup:

1. Articulated multibody AVBD extraction/lifecycle coverage.
2. Rigid contact persistence completeness.
3. Source-demo corpus implementation/optimization with real source packet
   evidence.
4. GPU row parity planning/evidence.

## Fresh-Session Handoff (2026-06-11)

North star: continue PLAN-104 AVBD toward source-shaped articulated rigid and
deformable row coverage with evidence against the native source corpus. Do not
count source-row overhead cleanup as a CPU-win, GPU, or paper-number gate; those
gates require dedicated corpus evidence.

Current resumed checkpoint: combined rigid linear/angular motor source-row
construction now has a scratch-taking overload for reusable large-input active
row pointer storage. The public overload keeps the local scratch fallback for
direct callers, while `AvbdRigidWorldContactSolveScratch` now owns and reserves
that motor-row scratch so `solveAvbdRigidWorldContactSnapshot()` does not build
temporary active linear/angular motor vectors on larger configured worlds. Local
validation for this slice passed
`pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`,
the focused
`AvbdRigidBlock.RigidMotorBuilderReusesLargeInputScratch:AvbdRigidBlock.RigidAngularMotorBuilderCreatesBoundedRows:AvbdRigidBlock.RigidLinearMotorBuilderCreatesBoundedRows:AvbdRigidBlock.RigidWorldSolveClearsAbsentRowFamilyInventories`
filter, the full `test_avbd_rigid_block` binary (96 tests),
`pixi run -- cmake --build build/default/cpp/Release --target test_world`, the
focused `World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap` no-heap
smoke, `pixi run lint`, `pixi run build`, and `git diff --check`. This remains
only source-row extraction overhead cleanup, not a source CPU-win, GPU, or
paper-number claim. No push, PR mutation, hosted CI rerun, merge, or branch
cleanup was performed.

Current resumed checkpoint: after the prior stop-only handoff, work resumed on
`avbd/source-row-extraction-precheck` to finish the contact-manifold small-row
scratch cleanup. `buildAvbdRigidContactManifoldRows()` now moves shared row
construction through a single active-contact append path and uses stack-backed
active contact, normal descriptor, cached local-anchor pair, and paired friction
descriptor storage when the contact count fits
`kAvbdRigidSmallRowStackCapacity`; larger manifolds keep using reusable
`AvbdRigidContactManifoldRowScratch` storage. A follow-up audit tightened the
small path so cached previous friction directions also use stack storage instead
of the scratch vector before `frictionInventory` is resynced. Local validation
for this slice passed `pixi run lint`, the affected rigid-block/contact target
build, the focused rigid contact-manifold filter (2 tests), the focused
boxed-LCP contact filter (17 tests), full `test_avbd_rigid_block` (95 tests),
full `test_boxed_lcp_contact` (122 tests), `pixi run build`, and
`git diff --check`. This checkpoint remains local; no push, PR mutation, hosted
CI rerun, merge, or branch cleanup was performed. Treat this branch as the
consolidated local resume branch and push only with explicit approval.

Latest literal stop handoff: on 2026-06-11 the user explicitly directed the
session to stop working further and only ensure the handoff docs. At that stop
point the active checkout was `avbd/source-row-extraction-precheck` at local
HEAD `04a369222a7 Record AVBD literal stop handoff`, ahead of
`origin/avbd/source-row-extraction-precheck` by three local commits:
`51eb9b48e08 Record AVBD angular motor validation evidence`,
`13604d8b8b5 Reuse AVBD distance spring row scratch`, and `04a369222a7`. Work
later resumed; use the current resumed checkpoint above as the current state.

Earlier literal stop handoff: on 2026-06-11 the user explicitly directed the
session to stop working further and only ensure the handoff docs. At that
earlier stop point the branch had two local commits on top of pushed
`origin/avbd/source-row-extraction-precheck`. Work later resumed; use the
current resumed checkpoint above as the current checkout state.

Immediate critical-stop handoff: on 2026-06-11 the user explicitly directed the
session to stop implementation and focus only on hand-off for all current work,
without any further verification. Preserve this branch state before doing new
implementation: the active checkout is `avbd/source-row-extraction-precheck`.
This handoff captures all current local state on the consolidated continuation
branch: the local point-joint source-row cleanup commit
`5a56acb0c4c Avoid point-joint row builder heap scratch for small AVBD inputs`
on top of pushed `f380bf9bc04`, the current standalone angular-motor source-row
cleanup in `rigid_block_kernel.hpp`, and these updated handoff docs. No
lint/build/test/CI or `git diff --check` was run after the stop instruction.
Validation listed below is evidence that existed before the stop instruction.
Do not apply local stashes by default; the branch itself is the consolidated
resume surface.

Current resumed validation note: work resumed after the no-verification
handoff, and the angular-motor source-row cleanup checkpoint
`ada568afa85 Checkpoint AVBD angular motor handoff state` has now passed the
focused rigid-block target, the focused angular-motor filter, full
`test_avbd_rigid_block` binary, `pixi run build`, `pixi run lint`, and
`git diff --check`. This validates only the narrow source-row scratch cleanup;
it does not close a source CPU-win, GPU, or paper-number gate.

Current resumed source follow-up: rigid distance-spring row extraction now has
`AvbdRigidDistanceSpringRowScratch` for the large-input active-row pointer list.
The existing small-input stack-backed path remains unchanged, while the world
contact solve scratch now owns and reserves this distance-spring scratch so
configured worlds can reuse it across frames like point-joint and motor row
scratch. Validation for this slice passed the focused rigid-block/world
targets, the focused distance-spring/motor rigid-block filter (4 tests), the
focused AVBD world filter (4 tests), full `test_avbd_rigid_block` (95 tests),
`pixi run build`, `pixi run lint`, and `git diff --check`. This is a narrow
source-row extraction cleanup and does not close any source CPU-win, GPU, or
paper-number gate.

Current resumed note: after the pushed handoff checkpoint at `7a9e24b487b`,
implementation resumed on `avbd/source-row-extraction-precheck` for small
source-row overhead slices. One committed local follow-up,
`e4b72c704d6 Skip origin-anchor point-pair direction cross products`, makes
generic rigid point-pair and paired friction direction assembly share the same
exact-origin helper as distance springs. The next checkpoint,
`52ccfd3187e Checkpoint AVBD source-row handoff state`, moved that helper
earlier so point attachments can use it, routes point-attachment and
distance-spring direction helpers through the same exact-origin path, and adds
`AvbdRigidBlock.PointAttachmentOriginAnchorDirectionStaysTranslational`. That
checkpoint has now been locally validated. This keeps source rows whose rigid
point-joint, motor, spring, attachment, or friction anchors sit exactly at a
body origin from computing angular-arm cross products in those helper paths. It
remains a narrow helper cleanup, not a CPU-win, GPU, or paper-number claim.
The later source edits introduce
`avbdRigidPointAttachmentConstraintValueAtWorldPoint()` and reuse a single
computed world anchor inside `addAvbdRigidPointAttachment()` for both constraint
value and direction assembly, avoid redundant tangent contact-anchor world
transforms by treating same-point tangent anchors as zero-relative-position, and
cache each active contact's body-local anchors once in
`buildAvbdRigidContactManifoldRows()` for reuse by both normal and paired
tangent friction rows. Recent local source-row cleanups avoid per-call heap
scratch for small point-joint and standalone angular-motor row builders by
using stack-backed active row/descriptor storage and source config pointers,
and add reusable large-input active-row scratch for rigid distance-spring row
extraction. The latest local checkpoint extends that small-input stack-backed
scratch pattern to `buildAvbdRigidContactManifoldRows()` by using stack storage
for active contacts, normal descriptors, cached local anchor pairs, and paired
friction descriptors when the contact count fits
`kAvbdRigidSmallRowStackCapacity`, and uses stack storage for the matching
previous-friction-direction cache in the small path. The existing reusable
scratch is retained for larger manifolds.
Before the critical stop, focused rigid-block checks, the full
`test_avbd_rigid_block` binary, `pixi run lint`, and `pixi run build` had
passed for earlier resumed source-row slices; the point-joint cleanup also
passed `pixi run build` and `git diff --check`. The angular-motor cleanup
has now passed the focused rigid-block target, the focused angular-motor
filter, full `test_avbd_rigid_block`, `pixi run build`, `pixi run lint`, and
`git diff --check`. `pixi run test-unit` had been started during an older
point-attachment handoff but no final result is claimed for that attempt.

Latest critical stop note: on 2026-06-11 the user explicitly redirected the
session to stop implementation and focus only on hand-off for all current work,
without any further verification. That checkpoint was committed and pushed
first. Work has since resumed, and the previously handoff-captured
point-attachment/distance-spring direction helper slice now has the local
validation evidence recorded below.

Historical stop note: an earlier session was also redirected to hand-off only
and requested no further verification. The branch was then resumed for a narrow
source-row prepare-overhead cleanup, redirected back to hand-off-only for a
docs-only pushed checkpoint, and resumed again for the narrow origin-anchor
helper fast paths now captured on the consolidated branch. Use the validation
section below for the current evidence, but do not treat hand-off commits as
new lint/build/test/CI evidence.

Current continuation state:

- Consolidated resume branch: use `avbd/source-row-extraction-precheck` for the
  next fresh Claude/Codex session. This branch is intended to be the single
  continuation branch for current AVBD source-row work; it contains the pushed
  #2977 parent changes, the stacked extraction-precheck changes, the current
  handoff docs, the formerly stash-only quaternion normalization fast path, the
  origin-anchor rigid world-point fast path, the origin-anchor distance-spring
  Hessian fast path, the origin-anchor distance-spring direction fast path, the
  generic point-pair/friction origin-anchor direction fast path, and the newest
  point-attachment/distance-spring direction helper slice, plus the newest
  point-attachment world-point reuse cleanup, the contact-manifold friction
  zero-relative-position cleanup, and the latest contact-manifold local-anchor
  reuse cleanup. The current resumed slice also makes small point-joint
  source-row builders use stack-backed active row/descriptor storage and pointer
  rows instead of copying full point-joint configs per active axis, extends the
  same pattern to standalone rigid angular-motor row construction, and adds
  reusable large-input scratch for rigid distance-spring source-row extraction.
  The latest local checkpoint also adds the contact-manifold small-input stack
  scratch refactor in `rigid_block_kernel.hpp`. The latest pushed continuation
  head is `ada568afa85` on `origin/avbd/source-row-extraction-precheck`; the
  local branch has additional source-row cleanup and handoff commits on top of
  that pushed head. Do not require a fresh session to inspect or apply local
  stashes before continuing. Keep this as the one consolidated local
  continuation branch for source-row cleanup until the user explicitly
  redirects or approves PR updates.
- The active PR is #2977,
  [`Trim AVBD source-row contact prep overhead`](https://github.com/dartsim/dart/pull/2977),
  on `avbd/source-row-perf-slice` at head `5297462d34b6118e600647cf18cdd7f13e0182b3`.
  It has been pushed and includes the merge from `origin/main` at
  `7d05d7b9ea7`.
- #2977 was refreshed read-only after this resumed slice. Its latest known
  state is open with merge state `BLOCKED` because one required hosted check
  was still running. CUDA Build, CI Lint, Alt Linux repro, CodeQL C++/Python,
  ReadTheDocs, Codecov project/patch, macOS arm64, Windows Release, wheels,
  Coverage Debug, Core Linux, Native Collision, Asserts, Eigen alignment,
  headless rendering, GUI smoke, and Linux Release Tests were
  green/skipped/neutral; Linux `Debug Tests` was still `IN_PROGRESS`. The
  Codecov comment at
  <https://github.com/dartsim/dart/pull/2977#issuecomment-4686080725>
  reported one uncovered patch line in `world_step_stage.cpp`, but the visible
  Codecov status checks were green. No new compiler, test, CodeQL, or blocking
  Codecov failure was visible on head
  `5297462d34b6118e600647cf18cdd7f13e0182b3`. Refresh this in a future session
  before making any PR decision.
- The active local checkout is `avbd/source-row-extraction-precheck`. The
  latest pushed continuation head is
  `ada568afa85 Checkpoint AVBD angular motor handoff state`; the local branch
  is ahead of that pushed head with the angular-motor validation,
  distance-spring scratch, literal stop handoff, and latest contact-manifold
  small-row scratch checkpoint. Earlier pushed heads in this stack included
  `8fc57deb9d6 Checkpoint AVBD handoff state`,
  `bf4a2cc8582 Record AVBD critical handoff state`, and contact-local-anchor
  parent `ae19e3b0822`. The current helper checkpoint sequence includes
  `52ccfd3187e Checkpoint AVBD source-row handoff state`,
  `63e3a1d44a1 Record AVBD source-row helper validation`,
  `8fc57deb9d6 Checkpoint AVBD handoff state`,
  `bf4a2cc8582 Record AVBD critical handoff state`, `ae19e3b0822`,
  `f380bf9bc04 Checkpoint AVBD contact local-anchor handoff`, `5a56acb0c4c`
  (point-joint small-row scratch cleanup), and `ada568afa85` (angular-motor
  small-row scratch cleanup).
- The earlier no-verification stop applied only to the previous handoff
  checkpoint. Work resumed afterward; the origin-anchor follow-ups have fresh
  local validation recorded below. No hosted CI rerun, PR comment, PR update,
  merge, or branch deletion was performed during this resumed slice before the
  current critical stop.
- Do not add unrelated commits to #2977 while it waits on hosted CI. If #2977
  needs fixes, keep them narrowly scoped to the PR branch and merge them back
  into the consolidated resume branch afterward.

Latest resumed local follow-up:

- `buildAvbdRigidContactManifoldRows()` now moves shared row construction into
  an internal `appendActiveRows` lambda and adds a small-input path backed by
  stack arrays for active contacts, normal descriptors, cached body-local
  anchor pairs, paired friction descriptors, and cached previous friction
  directions.
- Large contact manifolds still use `AvbdRigidContactManifoldRowScratch`; the
  scratch contact-local-point and friction-descriptor vectors are resized once
  and passed through the same shared row-construction path.
- Local validation for this slice passed: `pixi run lint`,
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block test_boxed_lcp_contact`,
  the focused rigid contact-manifold filter (2 tests), the focused boxed-LCP
  contact filter (17 tests), full `test_avbd_rigid_block` (95 tests), full
  `test_boxed_lcp_contact` (122 tests), `pixi run build`, and
  `git diff --check`.
- This is only a narrow source-row extraction cleanup. It does not close any
  source CPU-win, GPU, or paper-number gate.

Latest resumed local follow-up:

- `buildAvbdRigidDistanceSpringRows()` now has a scratch-backed overload for
  large-input active row pointers, while preserving the existing stack-backed
  small-input path.
- `AvbdRigidWorldContactSolveScratch` now owns and reserves
  `AvbdRigidDistanceSpringRowScratch`, and
  `solveAvbdRigidWorldContactSnapshot()` passes it into distance-spring row
  extraction so configured worlds can reuse that active-row storage across
  frames.
- Local validation for this slice passed:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block test_world`,
  the focused rigid-block distance-spring/motor filter (4 tests), the focused
  AVBD world filter (4 tests), full `test_avbd_rigid_block` (95 tests),
  `pixi run build`, `pixi run lint`, and `git diff --check`.
- This is only a narrow source-row extraction cleanup; it does not close any
  source CPU-win, GPU, or paper-number gate.

Latest resumed local follow-up:

- `buildAvbdRigidAngularMotorRows()` now mirrors the combined
  `buildAvbdRigidMotorRows()` path for small angular-motor inputs: active motor
  pointers and descriptors live on the stack when the possible row count fits
  `kAvbdRigidSmallRowStackCapacity`.
- `AvbdRigidAngularMotorRowScratch::activeRows` now stores pointers to input
  motors instead of copying full `AvbdRigidAngularMotor` configs into scratch.
  The active rows are consumed within the builder call.
- Local validation now completed for this slice:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`,
  the focused angular-motor filter
  `AvbdRigidBlock.RigidAngularMotorBuilderCreatesBoundedRows:AvbdRigidBlock.RigidAngularMotorRowsDriveTargetAngularStep`
  (2 tests), full `test_avbd_rigid_block` (95 tests), `pixi run build`,
  `pixi run lint`, and `git diff --check`.
- This is only a narrow source-row extraction cleanup; it does not close any
  source CPU-win, GPU, or paper-number gate.

Latest resumed local follow-up:

- `buildAvbdRigidPointJointRows()` and
  `buildAvbdRigidPointJointAngularRows()` now take the same small-row path used
  by adjacent motor/distance-spring builders: for up to
  `kAvbdRigidSmallRowStackCapacity` possible active axes, active row metadata
  and descriptors live on the stack instead of in scratch vectors.
- `AvbdRigidPointJointActiveAxis` now keeps a pointer to the input
  `AvbdRigidPointJoint` and an axis index instead of copying the full joint
  config once per enabled linear/angular axis. The rows are consumed within the
  builder call, so scratch does not become a persistent owner of joint data.
- Local validation for this slice passed:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`,
  the focused point-joint builder/config filter (11 tests), full
  `test_avbd_rigid_block` (95 tests), `pixi run lint`, `pixi run build`, and
  `git diff --check`.
- This is only a narrow source-row extraction cleanup. It does not close any
  source CPU-win, GPU, or paper-number gate.

Latest resumed local follow-up:

- `buildAvbdRigidContactManifoldRows()` now stores each active contact's
  body-local anchors in reusable scratch storage and reuses those anchors for
  both the normal row and the paired tangent friction rows. This avoids
  recomputing the same world-contact-point-to-body-local transforms in the
  friction row pass after the normal row pass already needed them.
- `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows` now also
  verifies that contact normal and friction rows share the generated local
  anchors for a non-origin contact.
- Local validation for this slice passed:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`,
  the focused
  `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows` filter,
  full `test_avbd_rigid_block` (95 tests), `pixi run lint`, and
  `pixi run build`.
- This is a narrow rigid contact source-row extraction cleanup. It does not
  close any source CPU-win, GPU, or paper-number gate.

Latest resumed local follow-up:

- `buildAvbdRigidContactManifoldRows()` now treats tangent friction anchors
  generated from the same manifold world contact point as having zero
  step-start relative displacement. This removes the redundant
  `avbdRigidPointPairRelativePosition()` call that transformed both freshly
  computed local anchors back to world space only to recover the same contact
  point pair.
- `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows` now sets
  nonzero body positions so the generated contact local anchors are nonzero,
  then verifies the manifold tangent rows still start with zero offsets and zero
  tangent constraint values.
- Local validation for this slice passed:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`,
  the focused
  `AvbdRigidBlock.RigidContactManifoldBuilderCreatesWarmStartedRows` filter,
  full `test_avbd_rigid_block` (95 tests), `pixi run lint`,
  `pixi run build`, and `git diff --check`.
- This is a narrow rigid contact source-row overhead cleanup. It does not close
  any source CPU-win, GPU, or paper-number gate.

Latest resumed local follow-up:

- `avbdRigidWorldPointDirection()` now centralizes 6D world-point direction
  assembly for exact body-origin anchors, returning a purely translational
  direction before computing any angular-arm cross product.
- `avbdRigidPointPairDirectionA/B()`, `addAvbdRigidPointPair()`,
  `addAvbdRigidPointPairFrictionTangentPair()`, and the serial
  `blockDescentRigidBodiesAvbdRows()` point-pair/friction assembly paths use
  that helper, so origin-anchor point-joint, motor, and friction rows avoid the
  same repeated angular cross products already skipped by distance springs.
- `AvbdRigidBlock.PointPairOriginAnchorDirectionStaysTranslational` verifies
  the new point-pair helper behavior. Focused target rebuild plus focused and
  full `test_avbd_rigid_block` validation passed before this docs update.
  `pixi run build` and `pixi run lint` also passed. This is a narrow source-row
  helper overhead cleanup; it does not close any source CPU-win, GPU, or
  paper-number gate.

Latest handoff-captured source edit:

- `avbdRigidPointAttachmentConstraintValueAtWorldPoint()` now evaluates a
  point-attachment scalar constraint from a precomputed world anchor.
- `addAvbdRigidPointAttachment()` computes `worldPoint` once, reuses it for the
  scalar constraint value, and passes the same point into
  `avbdRigidWorldPointDirection()`. This removes a duplicate
  `avbdRigidBodyWorldPoint()` evaluation in the point-attachment row stamping
  path while preserving `avbdRigidPointAttachmentConstraintValue(state, row)` as
  the public helper wrapper.
- Before the latest critical stop, this source edit had passed the focused
  rigid-block target/filter, the full `test_avbd_rigid_block` binary,
  `pixi run lint`, and `pixi run build`. `pixi run test-unit` had been started
  before the stop, but no final result is claimed in this handoff. No further
  verification was run after the stop instruction, so do not claim final
  post-stop lint, full-suite, or diff-check evidence for this handoff commit.
- This is only a narrow source-row helper overhead cleanup. It does not close
  any source CPU-win, GPU, or paper-number gate.

Newest validated local follow-up:

- `avbdRigidWorldPointIsBodyOrigin()` and `avbdRigidWorldPointDirection()` were
  moved immediately after `avbdRigidBodyWorldPoint()` so earlier helper paths
  can share them.
- `avbdRigidPointAttachmentDirection()` now builds the world point with
  `avbdRigidBodyWorldPoint()` and uses `avbdRigidWorldPointDirection()`, so an
  exact body-origin attachment anchor returns a purely translational direction.
- `avbdRigidPointPairDistanceSpringDirectionA/B()` now compute each endpoint
  world point and use `avbdRigidWorldPointDirection()`, preserving the existing
  off-origin angular cross-product path while skipping it for exact
  body-origin anchors.
- `AvbdRigidBlock.PointAttachmentOriginAnchorDirectionStaysTranslational` was
  added to cover the new point-attachment origin-anchor behavior.
- This slice has local validation evidence below. It remains only a narrow
  source-row helper overhead cleanup; it does not close any source CPU-win,
  GPU, or paper-number gate.

Previous resumed local follow-up:

- `avbdRigidWorldPointIsBodyOrigin()` centralizes the exact body-origin world
  anchor check used by distance-spring helpers.
- `avbdRigidDistanceSpringDirectionAtWorldPoint()` now returns a purely
  translational 6D direction for exact body-origin anchors and computes the
  angular arm cross product only for off-origin anchors.
- Both `addAvbdRigidPointPairDistanceSpring()` and the serial
  `blockDescentRigidBodiesAvbdRows()` distance-spring assembly path use that
  helper, so center-anchor Spring rows avoid repeated angular cross products in
  the same origin-anchor cases where the Hessian path already skips the generic
  world-point Jacobian multiply.
- `AvbdRigidBlock.DistanceSpringOriginAnchorDirectionStaysTranslational`
  verifies the new direction helper. The earlier
  `AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational` still
  covers the matching Hessian helper. This is a narrow source-row helper
  overhead cleanup; it does not close any source CPU-win, GPU, or paper-number
  gate.

Previous handoff-captured local follow-up:

- `addAvbdRigidDistanceSpringHessianAtWorldPoint()` now detects the exact
  body-origin anchor case (`worldPoint == state.position`) and directly stamps
  the translational 3x3 distance-spring Hessian block. For this case the
  rigid-world-point Jacobian is `[I, 0]`, so the generic
  `avbdRigidWorldPointJacobianAtWorldPoint()` call and 6x3 Hessian multiply are
  avoidable.
- `AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational` verifies
  a rotated body with a world-origin anchor receives only the translational
  Hessian block and no angular/translation-angular coupling.
- This targets center-anchor radial spring rows in the source Spring/Spring
  Ratio packets and is only a narrow helper overhead cleanup. It does not close
  any source CPU-win, GPU, or paper-number gate.
- Focused and full `test_avbd_rigid_block` validation passed before the final
  no-verification stop instruction. No lint/build/test/CI verification was run
  after this handoff-doc update by explicit user request.

Previous consolidated local follow-ups:

- `avbdRigidBodyWorldPoint()` now returns `state.position` directly for exact
  origin anchors. This avoids quaternion normalization and rotation in rigid
  source rows whose point constraints or radial distance springs attach at body
  origins, including the one-spring 2D Spring source row. The focused
  origin-anchor test and full rigid-block test binary pass. This is a narrow
  helper overhead cleanup; it does not close any source CPU-win, GPU, or
  paper-number gate.
- `RigidBodyContactStage::prepare()` now reserves AVBD scratch only when
  contacts could use `RigidAvbdContactConfig` storage or when point-joint /
  distance-spring AVBD pair constraints exist. Ordinary contact scratch still
  reserves for contact solving, but worlds with no possible AVBD rows avoid the
  AVBD snapshot/inventory/solve-scratch reserve path. This is a narrow
  source-row prepare-overhead cleanup; it does not close any source CPU-win,
  GPU, or paper-number gate.
- Rigid contact snapshot row assignment now avoids the duplicate
  `claimNextAvbdRigidWorldRow()` pass during contact collection. Contact rows
  are still canonicalized by endpoint pair and local point, then assigned in a
  single linear pass over the sorted row-order vector without writing the
  scratch row-counter vector.
- The unused local contact-row hash-map helper was removed from
  `rigid_world_contact.hpp`.
- This preserves warm-start row identity across contact/endpoint ordering and
  is only a narrow contact snapshot overhead cleanup. It does not close any
  source CPU-win, GPU, or paper-number gate.
- `normalizeAvbdRigidOrientation()` now uses `squaredNorm()` to reject invalid
  inputs before normalization, returns exact unit quaternions without division,
  and normalizes scaled finite inputs with `std::sqrt(squaredNorm)`.
- `AvbdRigidBlock.NormalizeRigidOrientationKeepsUnitAndRejectsInvalid` covers
  exact identity preservation, scaled normalization, zero fallback, and NaN
  fallback.
- This is a narrow repeated-helper overhead cleanup for rigid contact, joint,
  motor, and spring source rows. It does not close any source CPU-win, GPU, or
  paper-number gate.

Current #2977 technical scope:

- Keep the source-row optimization narrow: no-contact worlds still skip the
  contact query and collision-shape capacity scan during
  `RigidBodyContactStage::prepare()`.
- Preserve the contact-scene fix: when rigid contacts are possible,
  `prepare()` warms `World::queryContacts()` and reserves AVBD contact scratch
  from the larger of the collision-shape estimate and warmed query capacity.
- Do not reply to the Codecov bot comment
  `https://github.com/dartsim/dart/pull/2977#issuecomment-4686080725`. Treat
  it as addressed by the code/coverage-aware validation path and the next
  pushed coverage run.

CI diagnosis for the previous #2977 head `c4f8f5e3fd95`:

- `CUDA Build` failed on self-hosted runner `dartsim-mark13-4`. The job API
  showed step 8, `Build CUDA targets`, stuck `in_progress` with no completed
  failure step, and the log ended after `nvidia-smi`. No compiler or test
  failure was recorded.
- `Security | CodeQL (cpp)` failed on self-hosted runner `dartsim-mark13-5`.
  The job API showed step 8, `Build`, stuck `in_progress`; job logs were
  missing (`log not found`). No CodeQL finding or compiler error was recorded.
- The Codecov bot comment reported patch coverage at `95.65217%` with one
  missing changed line in `dart/simulation/compute/world_step_stage.cpp`, while
  all Codecov statuses were passing.

Local validation already run for #2977 head `5297462d34b`:

- `pixi run -e cuda -- cmake --build build/cuda/cpp/Release --target test_world`
  passed.
- `pixi run -e cuda -- bash -lc "build/cuda/cpp/Release/bin/test_world --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap' --gtest_brief=1"`
  passed in about 67 s.
- `pixi run build` passed.
- `pixi run test-unit` passed, 161/161 tests.
- `pixi run lint` passed before the merge commit on
  `avbd/source-row-extraction-precheck`.
- `pixi run -e cuda test-all` was started earlier on
  `avbd/source-row-extraction-precheck` and stopped intentionally after the user
  redirected the task to #2977; do not claim full CUDA `test-all` evidence from
  that aborted run.

Local validation for the consolidated branch after the latest source-row
cleanup:

- After the latest point-attachment world-point reuse source edit, but before
  the user's critical no-more-verification stop:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- After the latest point-attachment world-point reuse source edit:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.PointAttachmentStampsForceTorqueAndPsdHessian:AvbdRigidBlock.PointAttachmentOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.PointAttachmentDualUpdateGrowsInsideBounds:AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch:AvbdRigidBlock.BodyWorldPointKeepsOriginAnchorAtPosition' --gtest_brief=1"`
  passed, 5 tests.
- After the latest point-attachment world-point reuse source edit:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1"`
  passed, 95 tests.
- After the latest point-attachment world-point reuse source edit:
  `pixi run build` passed.
- After the latest point-attachment world-point reuse source edit:
  `pixi run test-unit` was started before the latest critical stop, but no
  final result is claimed in this handoff. Do not cite it as full-suite
  validation for this final state.
- After the user's critical stop, no additional lint/build/test/CI or
  `git diff --check` verification was run. In particular, `pixi run lint` was
  not rerun after the final handoff-doc edits by explicit user instruction.
- The previous handoff-only checkpoint intentionally ran no fresh verification.
  Work resumed afterward, and the current origin-anchor helper cleanups have
  fresh focused and suite validation below.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.PointAttachmentOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.PointAttachmentStampsForceTorqueAndPsdHessian:AvbdRigidBlock.PointPairOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.DistanceSpringOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational:AvbdRigidBlock.PointPairDistanceSpringStampsRadialFiniteStiffness:AvbdRigidBlock.PointPairDistanceSpringStepReducesStretch:AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows:AvbdRigidBlock.BodyWorldPointKeepsOriginAnchorAtPosition' --gtest_brief=1"`
  passed, 9 tests.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1"`
  passed, 95 tests.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint: `pixi run build` passed.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint: `pixi run lint` passed.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint: `git diff --check` passed.
- Current resumed validation after the point-attachment/distance-spring
  direction helper checkpoint: `pixi run test-unit` passed, 161/161 tests.
- Current resumed validation after the generic point-pair/friction
  origin-anchor direction helper:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- Current resumed validation after the generic point-pair/friction
  origin-anchor direction helper:
  `build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.PointPairOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.PointPairIncludesTorqueDirections:AvbdRigidBlock.PointPairStampsEqualAndOppositeRigidDirections:AvbdRigidBlock.DistanceSpringOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational:AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch:AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows' --gtest_brief=1`
  passed, 7 tests.
- Current resumed validation after the generic point-pair/friction
  origin-anchor direction helper:
  `build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1`
  passed, 94 tests.
- Current resumed validation after the generic point-pair/friction
  origin-anchor direction helper: `pixi run build` passed.
- Current resumed validation after the generic point-pair/friction
  origin-anchor direction helper: `pixi run lint` passed.
- Current resumed validation after the distance-spring origin-anchor direction
  helper:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- Current resumed validation after the distance-spring origin-anchor direction
  helper:
  `build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.DistanceSpringOriginAnchorDirectionStaysTranslational:AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational:AvbdRigidBlock.PointPairDistanceSpringStampsRadialFiniteStiffness:AvbdRigidBlock.PointPairDistanceSpringStepReducesStretch:AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch:AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows:AvbdRigidBlock.BodyWorldPointKeepsOriginAnchorAtPosition' --gtest_brief=1`
  passed, 7 tests.
- Current resumed validation after the distance-spring origin-anchor direction
  helper:
  `build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1`
  passed, 93 tests.
- Current resumed validation after the distance-spring origin-anchor direction
  helper: `git diff --check` passed.
- Current resumed validation after the distance-spring origin-anchor direction
  helper: `pixi run lint` passed.
- Current resumed validation after the distance-spring origin-anchor direction
  helper: `pixi run build` passed.
- Current resumed validation after the distance-spring origin-anchor direction
  helper: `pixi run test-unit` passed, 161/161 tests.
- Current resumed validation after the distance-spring Hessian origin-anchor
  fast path, before the final no-verification stop:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- Current resumed validation after the distance-spring Hessian origin-anchor
  fast path:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.DistanceSpringOriginAnchorHessianStaysTranslational:AvbdRigidBlock.PointPairDistanceSpringStampsRadialFiniteStiffness:AvbdRigidBlock.PointPairDistanceSpringStepReducesStretch:AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch:AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows:AvbdRigidBlock.BodyWorldPointKeepsOriginAnchorAtPosition' --gtest_brief=1"`
  passed, 6 tests.
- Current resumed validation after the distance-spring Hessian origin-anchor
  fast path:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1"`
  passed, 92 tests.
- Current resumed validation before the final handoff-doc edit:
  `git diff --check` passed.
- Current resumed validation before the final handoff-doc edit:
  `pixi run lint` passed.
- Current resumed validation after the origin-anchor world-point fast path:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- Current resumed validation:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.BodyWorldPointKeepsOriginAnchorAtPosition:AvbdRigidBlock.PointPairDistanceSpringStepReducesStretch:AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch:AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows' --gtest_brief=1"`
  passed, 4 tests.
- Current resumed validation:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1"`
  passed, 91 tests.
- Current resumed validation: `pixi run lint` passed.
- Current resumed validation: `pixi run build` passed.
- Current resumed validation: `git diff --check` passed.
- Current resumed validation: `pixi run test-unit` was attempted after the
  origin-anchor commit. It built the C++ unit/integration test binaries and
  entered CTest, but the host load spiked to about 48 and CTest waited behind
  `--test-load 22` without starting child test processes. The blocked attempt
  was stopped; do not claim full `test-unit` evidence for this slice from that
  run.
- Current resumed validation after the prepare-stage AVBD scratch reserve guard:
  `pixi run -- cmake --build build/default/cpp/Release --target test_world`
  passed.
- Current resumed validation:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity:World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet:World.RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide' --gtest_brief=1"`
  passed, 4 tests.
- Current resumed validation:
  `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- Current resumed validation:
  `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows:AvbdRigidBlock.NormalizeRigidOrientationKeepsUnitAndRejectsInvalid:AvbdRigidBlock.RigidWorldContactSnapshotRowsIgnoreContactOrder:AvbdRigidBlock.RigidWorldContactSnapshotRowsIgnoreEndpointOrder' --gtest_brief=1"`
  passed, 4 tests.
- Current resumed validation: `pixi run lint` passed.
- Current resumed validation: `pixi run build` passed.
- Current resumed validation: `git diff --check` passed.
- Current resumed validation: `pixi run test-unit` was attempted and built the
  C++ unit/integration test binaries, then entered CTest, but the host load was
  about 52 and the DART CTest wrapper waited behind `--test-load 22` without
  starting child test processes. The blocked full-suite attempt was stopped;
  do not claim full `test-unit` evidence for this slice from that run.
- `git diff --check` passed.
- `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed after the contact row-assignment cleanup.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.RigidWorldContactSnapshotRowsIgnoreContactOrder:AvbdRigidBlock.RigidWorldContactSnapshotRowsIgnoreEndpointOrder' --gtest_brief=1"`
  passed, 2 tests.
- The same focused target rebuild and row-order filter passed again after
  removing contact-assignment scratch row-counter writes.
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run -- cmake --build build/default/cpp/Release --target test_avbd_rigid_block`
  passed.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.NormalizeRigidOrientationKeepsUnitAndRejectsInvalid' --gtest_brief=1"`
  passed.
- `pixi run -- bash -lc "build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_brief=1"`
  passed, 90 tests.
- `pixi run test-unit` was attempted, but the host load was about 50 and the
  DART CTest wrapper waited behind `--test-load 22` without starting child test
  processes. The run was stopped rather than left running in the background; do
  not claim final consolidation `test-unit` evidence from that attempt.

Local branch inventory at this handoff:

| Branch                                 | Upstream                                      | Local head at handoff   | State and handling                                                                                |
| -------------------------------------- | --------------------------------------------- | ----------------------- | ------------------------------------------------------------------------------------------------- |
| `avbd/source-row-extraction-precheck`  | `origin/avbd/source-row-extraction-precheck`  | Latest local checkpoint | Current checkout; ahead of origin with local source-row cleanup commits. Push only with approval. |
| `avbd/source-row-perf-slice`           | `origin/avbd/source-row-perf-slice`           | `5297462d34b`           | Active #2977 branch; pushed, latest known state was waiting on Linux Debug Tests.                 |
| `avbd/articulated-stiffness-roundtrip` | `origin/avbd/articulated-stiffness-roundtrip` | `43787619654`           | #2975-era branch; PR is reported merged. Candidate for cleanup after confirmation.                |
| `feature/avbd-articulated-masked-rows` | `origin/feature/avbd-articulated-masked-rows` | `d25e5177d9c`           | Raw 33-hour safety checkpoint. Keep until all split AVBD slices are safely landed.                |
| `feature/free-joint-energy-benchmarks` | `origin/feature/free-joint-energy-benchmarks` | `d13c97b5f0c`           | Unrelated local branch; do not touch during AVBD handoff.                                         |
| `main`                                 | `origin/main`                                 | `7d05d7b9ea7`           | Matched fetched `origin/main` at latest checked base. Refresh before using.                       |

No local branch deletion or remote branch cleanup was performed during the
latest critical handoff-only stop. The branches above are intentionally left in
place. Cleanup candidates remain documented here, but a future session should
ask before deleting local branches, remote branches, or stashes.

Remote-only AVBD split branches still visible:

- `origin/avbd/core-row-solver` at `b8e34e4c495`.
- `origin/avbd/world-integration-serialization` at `2af5f83c88a`.
- `origin/avbd/variational-integrator-extensions` at `d826d010991`.

Local stashes at this handoff:

- `stash@{0}` on `avbd/source-row-extraction-precheck`:
  `codex-pr2977-switch-preserve-extraction-precheck-wip`. It touches
  `world_step_stage.cpp`, `rigid_block_kernel.hpp`, this dev-task README/RESUME,
  and `test_avbd_rigid_block.cpp`. The `world_step_stage.cpp` hunk is the
  contact-query warmup already present in #2977. The quaternion
  normalization/test content has now been folded into the consolidated branch.
  Treat this stash as a historical recovery point, not required resume input.
- `stash@{1}` on `avbd/source-row-extraction-precheck`:
  `codex-avbd-normalize-fastpath-wip`. It contains the same quaternion
  normalization fast path/test without the #2977 contact-query overlap; this has
  now been folded into the consolidated branch. Treat it as historical only.
- `stash@{2}` on `feature/avbd-articulated-masked-rows`:
  `codex-avbd-pre-https-origin-main-merge-20260609220658`.
- `stash@{3}` on `feature/avbd-articulated-masked-rows`:
  `codex-temp-avbd-before-origin-main-merge`.
- `stash@{4}` on `feature/avbd-articulated-masked-rows`:
  `codex-avbd-pre-main-merge-20260609074526`.
- `stash@{5}` on `feature/avbd-articulated-masked-rows`:
  `codex-avbd-pre-main-merge-20260609060120`.
- `stash@{6}` on `feature/avbd-articulated-masked-rows`:
  `codex-avbd-pre-main-merge-20260609005244`.
- `stash@{7}` on `feature/avbd-articulated-masked-rows`:
  `codex-avbd-pre-main-merge-20260608225323`.

Fresh-session plan after this progress checkpoint:

1. Start with `git switch avbd/source-row-extraction-precheck`,
   `git status --short --branch`, and read this file before doing any work.
   The branch intentionally has local commits beyond origin unless a later
   session pushes them with explicit approval. Do not apply local stashes by
   default.
2. If the next session is asked to resume PR/CI work, then fetch and refresh:
   `git fetch origin main` (or the equivalent HTTPS fetch if SSH to GitHub is
   still blocked on port 22) and
   `gh pr view 2977 --json mergeStateStatus,headRefOid,statusCheckRollup`.
3. The newest contact-manifold small-row scratch cleanup has local validation
   recorded above. Before reviewer-facing PR work, refresh against the intended
   base and rerun the appropriate lint/build/test checks if source changes are
   added. Keep source-row overhead claims separate from CPU-win, GPU, and
   paper-number gates.
4. If #2977 CI has failed, inspect only the newest failed run/job and keep any
   fix limited to the prepare/cache-reserve behavior unless CI proves a separate
   issue. Run `pixi run lint` before committing.
5. If #2977 is green and mergeable, ask before merging; do not merge without
   explicit user approval.
6. After #2977 lands, ask before branch cleanup. Likely cleanup candidates are
   `avbd/source-row-perf-slice` and
   `avbd/articulated-stiffness-roundtrip`; keep
   `feature/avbd-articulated-masked-rows` as the raw checkpoint until the full
   split AVBD work is safely landed, and leave
   `feature/free-joint-energy-benchmarks` alone.
7. Continue stacked work on `avbd/source-row-extraction-precheck` only after
   merging the latest parent branch. Do not apply local stashes by default; the
   known still-relevant quaternion hunk has already been committed to the
   consolidated branch, and the duplicate `world_step_stage.cpp` hunk should be
   skipped.
8. Keep updating this handoff with every plan/progress change that affects the
   next session, including branch heads, stashes, PR state, validation, and
   cleanup decisions.

## Last Session Summary

Latest local follow-up: PLAN-091 WP-091.1 relabels the AVBD contact-scene
evidence rows: no `avbd-demo2d`/`avbd-demo3d` benchmark or py-demo scene
emplaces the internal AVBD rigid-contact opt-in config
(`comps::RigidAvbdContactConfig`), because AVBD contact is not
facade-selectable, so every rigid contact in those scenes ran DART's default
sequential-impulse contact path. The pure-contact rows (2D Dynamic Friction,
Static Friction, Pyramid, Cards, Stack, and Stack Ratio; 3D Ground, Dynamic
Friction, Static Friction, Pyramid, Stack, and Stack Ratio) timed no AVBD rows
at all; the joint-plus-contact rows (2D Fracture, Soft Body, Joint Grid, and
Net; 3D Soft Body, Bridge, and Breakable) timed AVBD point-joint/motor/spring
rows while their ordinary contacts ran sequential impulse; incidental link-link
contacts in the chain rows (2D Rod, Rope, Heavy Rope, and Hanging Rope; 3D Rope
and Heavy Rope) also ran sequential impulse. Their faster/slower-than-native
ratios are whole-pipeline `World::step` comparisons, not AVBD-contact-solver
comparisons. New AVBD evidence packets must machine-record
`resolved_solver_identity` at AVBD packet schema version 2
(`scripts/avbd_packet_schema.py`, enforced by `pixi run check-avbd-packets`);
committed packet bytes are unchanged. This is an evidence-integrity relabel
only; it does not close or reopen any AVBD solver, source-corpus CPU-win, GPU,
or paper-number gate.

Latest local follow-up: `RigidBodyContactStage::execute()` now uses
storage-level AVBD point-joint and distance-spring prechecks before running the
exact extraction views. This avoids scanning the same AVBD config sets twice in
joint/spring source rows while still returning early without scratch allocation
in worlds that have no private pair-constraint storage. Focused
`World.RigidBodyContactStageAvbd*`,
`AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows`,
`AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch`,
`AvbdRigidBlock.RigidWorldContactStep*`, and selected dartpy world tests pass.
A selected same-filter benchmark toggle under high host load moved median CPU
time lower for 2D Motor, Joint Grid, Rope, Heavy Rope, Hanging Rope, Spring,
Spring Ratio, 3D Rope, and 3D Heavy Rope, while 3D Spring and Spring Ratio were
slightly slower in the noisy smoke. This remains source-row overhead evidence
only and does not close any CPU-win, GPU, or paper-number gate.

Latest handoff follow-up: PR #2977 (`avbd/source-row-perf-slice`) has been
merged forward to `origin/main` at `7d05d7b9ea7` after the hosted PR branch
reported `BEHIND`. The two red hosted checks were self-hosted runner losses
during build (`CUDA Build` on `dartsim-mark13-4` and CodeQL C++ on
`dartsim-mark13-5`); their logs stop with the build step still in progress and
no compiler/test failure recorded. The Codecov bot comment on
`world_step_stage.cpp` reported one uncovered changed line while the Codecov
statuses were passing. The PR follow-up keeps the no-contact prepare skip, but
restores prepare-time `World::queryContacts()` cache warmup for worlds that can
have rigid contacts and sizes AVBD contact scratch from the larger of the
collision-shape estimate and warmed query capacity. Focused CUDA
`test_world --gtest_filter=World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap`,
`pixi run build`, and `pixi run test-unit` pass locally. Full CUDA `test-all`
was previously started on the stacked extraction branch and intentionally
stopped when the task changed to #2977 handoff. This remains no-contact/contact
prepare-overhead evidence only and does not close any CPU-win, GPU, or
paper-number gate.

Latest local follow-up: after merging `origin/main` at `906a6c0241fe`, the
narrow `RigidBodyContactStage::prepare()` source-row cleanup still builds and
focused AVBD source-row tests plus `pixi run test-unit` pass. The cleanup avoids
the collision-shape capacity scan when the contact query is already skipped,
avoids running a prepare-time contact query that `execute()` repeats only to size
AVBD contact scratch, and uses the distance-spring storage size for AVBD scratch
reserve capacity instead of iterating the spring configs just to count them.
Earlier same-branch benchmark smoke under lower host load moved
`BM_AvbdDemo2dMotorStep_median` from about 9.34 us to 8.83 us,
`BM_AvbdDemo2dSpringStep_median` from about 5.04 us to 4.28 us, and
`BM_AvbdDemo2dSpringRatioStep_median` from about 45.3 us to 37.1 us. Coverage
CI then caught that the no-query cleanup still needed contact-scene AVBD scratch
reservations, so the kept variant reserves contact scratch from the
collision-shape capacity estimate without restoring the duplicate
`queryContacts()` call. This remains no-contact/source-row overhead evidence and
does not close any source CPU-win, GPU, or paper-number gate.
Latest local follow-up: C++ and dartpy public articulated AVBD stiffness
persistence coverage now exercises fixed, revolute, prismatic, and spherical
public articulated facades for both same-multibody link pairs and world-link
endpoints. The C++ serialization test and
`test_simulation_world_articulated_avbd_stiffness_roundtrip_from_python` set
finite start/linear/angular AVBD stiffness, save/load the world, mutate the
restored stiffness values, and verify they remain visible after entering
simulation mode. A focused variational-integration test now also verifies
restored stiffness for those endpoint/type combinations feeds the private
point-joint configs rebuilt at simulation entry. This is narrow public facade
serialization/extraction coverage; it does not close broader articulated
lifecycle, source-corpus CPU-win, GPU, or paper-number gates.

Latest local follow-up: CUDA boxed-LCP PGS dense world-contact tests now keep
the largest 128-box fixture as a cheap shape gate while bounding the default
runtime-contract CUDA sweeps to smaller dense packets. #2973's CUDA failure
timed out in the pre-existing `test_lcp_jacobi_batch_cuda` dense PGS suite;
`main` passed the same binary but only with about 1099 seconds of runtime. This
is CI-runtime calibration only; it does not close any AVBD solver, CPU-win,
GPU, or paper-number gate.

Latest local follow-up: the large BoxedLcp dense/articulated scaling packets in
`test_boxed_lcp_contact` are now opt-in behind
`DART_BOXED_LCP_CONTACT_ENABLE_EXPENSIVE_SCALING_TESTS`, while representative
default coverage still includes dense-contact shape smoke and small/mid-size
articulated public-step cases. This keeps routine CI from spending minutes in
the dense fallback path while preserving the larger packets for dedicated
evidence/performance runs. `pixi run lint`, focused CTest, and the dartpy world
test passed locally. This is CI-runtime calibration only; it does not close any
AVBD solver, CPU-win, GPU, or paper-number gate.

Latest local follow-up: dartpy public articulated fixed, spherical, cardinal
one-DOF motor, and non-cardinal one-DOF motor break/reset coverage now rechecks
endpoint shape, joint type, DOF count, and motor axis after reset re-engages the
rows for same-multibody, world-link, and movable-pair cases; the non-cardinal
direct cases now also recheck that shape while broken rows are being skipped
before reset. Broken-state binary round-trip tests now also recheck the
restored facade shape after reset re-engages same-multibody/world-link fixed,
spherical, and one-DOF rows. The focused selected pytest filters passed. This
is only a narrow public facade lifecycle assertion slice; it does not close
broader articulated fracture, motor lifecycle, source-corpus, CPU-win, GPU, or
paper-number gates.

Latest local follow-up: small AVBD rigid world-contact snapshots no longer
reserve the endpoint entity-index hash map while the body count is within the
small-row linear-scan capacity. Focused `test_avbd_rigid_block` snapshot-index
validation passed, and a selected local Motor benchmark smoke moved from the
earlier about 9.3 us mean to about 8.0 us mean CPU time under host load around 7. This is only no-contact source-row overhead evidence; it does not close any
source-row CPU-win, GPU, or paper-number gate.

Latest local follow-up: C++ and dartpy public articulated facade coverage now
verifies that `World::clear()` / `World.clear()` invalidates existing
articulated link/joint handles, drops the public point-joint facades, and resets
the generated `joint_###` name sequence before a rebuilt world creates a fresh
empty-name facade. This is only a narrow public facade lifecycle/name-counter
guard; it does not close broader articulated motor lifecycle, fracture corpus,
CPU-win, GPU, or paper-number gates.

Latest local follow-up: Linux Release CI now runs the ASAN build with
`DART_PARALLEL_JOBS=4` after the hosted runner killed the ASAN compilation with
exit 137 while rebuilding the full test target set. This is CI resource
calibration only; it does not close any AVBD solver, CPU-win, GPU, or
paper-number gate.

Latest local follow-up: the AVBD-only contact regressions in
`test_boxed_lcp_contact` are excluded on MSVC after both target-level `/Od` and
a source-local optimization guard still hit C1001 in MSVC 19.44. The ordinary
BoxedLcp contact tests remain active on Windows, and the AVBD regressions remain
active on Linux/macOS. This is CI portability only; it does not close any AVBD
solver, CPU-win, GPU, or paper-number gate.

Latest local follow-up: the IPC bake allocation regression now compares the
unsupported plane scene against the same IPC contact-query-only setup with
supported collision geometry instead of against the sequential solver's
allocation baseline. This keeps the guard focused on accidental IPC query
prewarm work after the sequential prepare path was tightened. This is CI-test
calibration only; it does not close any AVBD solver, CPU-win, GPU, or
paper-number gate.

Latest local follow-up: `test_boxed_lcp_contact` now avoids a repeated MSVC
19.44 internal compiler error in the primitive endpoint-row helper by using an
explicit `PrimitiveRowKey` struct instead of a nested
`std::pair<AvbdContactEndpointId, AvbdContactEndpointId>` alias in that
section. The focused Linux target rebuild and full `test_boxed_lcp_contact`
binary passed locally. This is CI portability only; it does not close any AVBD
solver, CPU-win, GPU, or paper-number gate.

Latest local follow-up: `RigidBodyContactStage::prepare()` now checks the same
no-dynamic-collision-geometry predicate as execute before sizing contact
scratch, avoiding a redundant `World::queryContacts()` call for no-collision
source rows such as the 2D Motor row. Focused world/contact-stage tests and
step-profiling tests passed, and a selected Motor benchmark smoke moved from
about 13.5 us to about 9.2 us median CPU under changing host load. This is
no-contact prepare-overhead evidence only; it does not close the Motor CPU-win,
GPU, or paper-number gates.

Latest local follow-up: point-pair and angular constraint caches now use exact
component equality for repeated local anchors and target orientations instead
of zero-tolerance approximate comparisons. This keeps the block-solver cache-hit
path aligned with how joint rows copy shared descriptors. Focused
`test_avbd_rigid_block` validation passed. A selected source-row benchmark smoke
ran under host load average around 12.8 and is treated only as noisy overhead
evidence; source-row CPU-win, GPU, and paper-number gates remain open.

Latest local follow-up: the rigid block solver's inlined angular row-state
update now skips capped finite-stiffness angular rows before recomputing
orientation error, matching the existing standalone angular updater and finite
point-pair fast path. Focused `test_avbd_rigid_block` validation passed. A
same-session selected source-row benchmark smoke ran under high host load and
is treated only as overhead evidence; source-row CPU-win, GPU, and paper-number
gates remain open.

Latest local follow-up: after fetching `origin/main` through HTTPS and
fast-forwarding the branch to `f1fa9f386f9`, rigid AVBD point-pair rows now
avoid computing `previousConstraintValue` for finite-stiffness joint rows and
reuse consecutive point-pair world-anchor transforms during block assembly and
row-state updates. Focused `test_avbd_rigid_block` validation passed, and a
short selected-row benchmark smoke recorded `BM_AvbdDemo2dSoftBodyStep_median`
at about 2.03 ms plus Rod/Joint Grid/Soft Body/Rope/Net at about
0.149 ms / 10.8 ms / 1.70 ms / 0.113 ms / 0.360 ms under load. This is local
overhead evidence only; the 2D Soft Body packet still does not beat the native
source runner, and CPU-win, GPU, and paper-number gates remain open.

Latest local follow-up: dartpy public articulated joint-list lifetime coverage
now has `test_simulation_world_articulated_joint_list_keeps_world_alive`. It
returns link-link and world-link articulated facade handles from a temporary
World, forces Python garbage collection, then verifies endpoint access still
keeps the original World alive. This is only a narrow dartpy public API lifetime
guard and does not close broader articulated motor lifecycle, CPU-win, GPU, or
paper/source-runner gates.

Latest local follow-up: public articulated facade generated-name persistence
now has
`Serialization.PreservesJointCounterAcrossArticulatedGeneratedNames` and
`test_simulation_world_articulated_generated_names_resume_after_binary_roundtrip`.
They save explicit `joint_001` plus generated `joint_002`, reload them, and
prove the next empty-name public articulated facade advances past the restored
topology joint component to `joint_004` instead of reusing an existing public
articulated joint name. This is only a narrow
serialization/name-counter robustness guard and does not close broader
articulated motor lifecycle, CPU-win, GPU, or paper/source-runner gates.

Latest local follow-up: public articulated facade generated-name coverage now
has `World.ArticulatedPointJointsGenerateUniqueFacadeNames` and
`test_simulation_world_articulated_point_joints_generate_unique_names`. They
seed an explicit `joint_001`, create empty-name fixed/revolute/prismatic
non-topology articulated facades, and verify generated names skip the existing
joint, appear through articulated name/list lookups, and reject duplicates.
This is only a narrow public facade/corpus robustness guard and does not close
broader articulated motor lifecycle, CPU-win, GPU, or paper/source-runner
gates.

Latest local follow-up: dartpy public articulated lifecycle coverage now has
`test_simulation_world_clear_invalidates_articulated_joint_handles_from_python`.
It steps a world-link articulated revolute motor through Python, calls
`World.clear()`, verifies the old multibody/link/joint handles plus
articulated-joint name/list lookups are invalidated, then rebuilds a fresh
world-link prismatic motor and verifies it steps along its free axis. This is
only a narrow Python public API clear/rebuild guard and does not close broader
articulated motor lifecycle, CPU-win, GPU, or paper/source-runner gates.

Latest local follow-up: public world-link articulated AVBD lifecycle coverage
now has
`VariationalIntegration.AvbdPublicArticulatedWorldFacadeClearDropsExtractedRowsAndRebuilds`.
It extracts private rows for a world-link revolute facade, steps it, calls
`World::clear()`, verifies the old public handles and facade state are gone,
then rebuilds a fresh world-link prismatic facade and verifies its private rows
extract and drive only the free slider axis. This is only a narrow
clear/rebuild lifecycle guard and does not close broader articulated motor
lifecycle, CPU-win, GPU, or paper/source-runner gates.

Latest local follow-up: public world-link articulated motor demo coverage now
explicitly guards the world-prismatic and world-revolute break/reset scenes as
child-only world-endpoint facades from Python, and the
`avbd-articulated-world-prismatic-breakable-motor-packet.json` remaining gates
no longer list the already-staged world-revolute packet. This is only a narrow
demo/packet consistency guard and does not close broader articulated motor
lifecycle, CPU-win, GPU, or paper/source-runner gates.

Latest local follow-up: live contact-stage AVBD friction coverage now reaches
the two-lower, two-upper box-pile topology through
`World.RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionReducesSlip`
and
`World.RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionRunsThroughDefaultWorldStep`.
They configure both lower supports and both independent upper boxes for private
AVBD contact, then verify warm-started friction reduces each upper/lower
contact-point slip in both the explicit `RigidBodyContactStage` path and the
built-in `World::step()` schedule. This closes only a narrow live multi-top
box-pile contact-stage/default-step friction slice and no broad pile/contact
persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: private stacked static/dynamic plus dynamic/dynamic
box-manifold contact-order replay now has
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreContactOrder`.
It reverses the live stacked `World::collide()` contact vector and verifies
canonical endpoint rows, per-pair friction coefficients, Coulomb bounds, and
warm-started normal/friction lambdas survive raw narrow-phase contact-order
changes. This closes only a private stacked-manifold contact-order slice and no
broad pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: private rigid-contact multi-top pile endpoint-order
replay now has
`AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsIgnoreEndpointOrder`.
It swaps every raw contact endpoint for the two-lower, two-upper box pile and
verifies canonical endpoint rows, distinct per-pair friction coefficients,
Coulomb bounds, and paired tangent-dual projection survive swapped physical
tangent bases. This closes only a private multi-top box-pile endpoint-order
slice and no broad pile/contact persistence, CPU-win, GPU, or paper-number
gate.

Latest local follow-up: private rigid-contact multi-top pile contact-order
replay now has
`AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsIgnoreContactOrder`.
It reverses the live `World::collide()` contact vector for a two-lower,
two-upper box pile and verifies canonical endpoint rows, row ordinals, distinct
per-pair friction coefficients, Coulomb bounds, and warm-started normal/friction
lambdas survive raw narrow-phase contact-order changes. This closes only a
private multi-top box-pile contact-order slice and no broad pile/contact
persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: generated current-pose same-multibody prismatic
finite-force coverage now uses a non-cardinal slider basis in
`VariationalIntegration.AvbdPrismaticPointJointCurrentPoseExtractorRespectsTinyForceLimitOnMovablePair`.
This proves the generated hard rows preserve the captured pose while the finite
motor row remains bounded off the cardinal axes. Focused validation passed for
this newest slice. This closes only a narrow generated prismatic
movable-pair finite-force coverage slice and no broad articulated fracture,
motor-lifecycle, CPU-win, GPU, or paper-number gate.

Recent local follow-up: direct private fixed world-link save/load/reset now has
`VariationalIntegration.AvbdBreakableParentPointJointConfigResetReengagesFixedRows`.
It covers the missing floating parent-link endpoint polarity with an off-origin
anchor and non-identity target orientation, preserves the broken-state
`AvbdRigidWorldPointJointConfig` across binary load, and proves reset
re-engages all fixed rows. The branch was first fast-forwarded to
`origin/main` at `dbac6c63e9f`; the `World::clear()` conflict was resolved by
keeping main's storage recreation and the AVBD variational-integrator defaults.
`pixi run lint`, the focused `test_variational_integration` rebuild, and the
five-test fixed/spherical save-load-reset endpoint-polarity filter passed
locally. This closes only a narrow direct private fixed world-link polarity
slice and no broad articulated fracture, motor-lifecycle, CPU-win, GPU, or
paper-number gate.

Recent local follow-up: private generated fixed/spherical current-pose
extraction now has
`VariationalIntegration.AvbdFixedPointJointCurrentPoseExtractorWiresParentWorldLinkEndpoint`
and
`VariationalIntegration.AvbdSphericalPointJointCurrentPoseExtractorWiresWorldLinkEndpointPolarities`.
They cover parent-link world-link endpoint polarity for non-topology private
fixed and spherical point joints (plus the spherical child-link polarity),
verify the generated anchor storage and all-axis fixed or linear-only
spherical row masks, and prove the variational solve pins the extracted anchor
while leaving spherical orientation free. The focused filters passed locally.
This closes only narrow private fixed/spherical world-link current-pose
polarity slices and no broad articulated fracture, motor-lifecycle, CPU-win,
GPU, or paper-number gate.

Latest local diagnostic: a conservative small-scene broad-sphere precheck before
`RigidBodyContactStage` contact queries was prototyped for the open
Spring/Spring Ratio source-row CPU gap and then reverted. The same-session
focused `BM_AvbdDemo(2d|3d)Spring*` smoke moved the medians from about
3.79 us / 34.26 us / 3.90 us / 28.94 us to about
4.01 us / 33.51 us / 4.44 us / 29.75 us for 2D Spring, 2D Spring Ratio,
3D Spring, and 3D Spring Ratio respectively, so the result was mixed and did
not justify carrying an extra pre-query bound pass. Treat this as rejected
non-packet performance evidence only; the Spring/Spring Ratio CPU-win and GPU
gates remain open.

Latest local follow-up: private rigid-contact pile row-persistence coverage now
has
`AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsPersistAcrossSmallPose`.
It keeps two lower dynamic supports on static ground plus two independent upper
dynamic bodies active in one live pile and verifies stable canonical
endpoint-key row ordinals, distinct per-pair friction coefficients, Coulomb
bounds, warm-started normal lambdas, and paired friction duals across a small
same-feature pose nudge. The focused `AvbdContact.WorldCollide*` filter passed
40/40 after lint. This closes only a private multi-top box-pile row-persistence
slice and no broad pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: mesh private rigid-contact endpoint-order coverage now
has `AvbdContact.WorldCollideMeshFaceRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideMeshEdgeRowsIgnoreEndpointOrder`, and
`AvbdContact.WorldCollideMeshVertexRowsIgnoreEndpointOrder`. They swap raw
endpoint order for live sphere/mesh face, edge, and vertex contacts and verify
stable canonical endpoint-key row ordinals, mesh feature IDs, Coulomb bounds,
warm-started normal lambdas, and projected friction duals. The focused
`AvbdContact.WorldCollide*` filter passed 39/39 after lint. This closes only
mesh endpoint-order coverage and no broad pile/contact persistence, CPU-win,
GPU, or paper-number gate.

Latest local follow-up: cylinder/capsule primitive rigid-contact endpoint-order
coverage now has
`AvbdContact.WorldCollideCylinderCapRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideCylinderSideRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideCylinderRimRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideCapsuleCapRowsIgnoreEndpointOrder`, and
`AvbdContact.WorldCollideCapsuleSideRowsIgnoreEndpointOrder`. They swap raw
endpoint order for live cap/side/rim contacts and verify stable canonical
endpoint-key row ordinals, primitive feature IDs, Coulomb bounds, warm-started
normal lambdas, and projected friction duals. The focused
`AvbdContact.WorldCollide*` filter passed 36/36 after lint. This closes only
primitive endpoint-order coverage and no broad
pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: cylinder/capsule primitive rigid-contact small-pose
persistence now has
`AvbdContact.WorldCollideCylinderCapRowsPersistAcrossSmallPose`,
`AvbdContact.WorldCollideCylinderSideRowsPersistAcrossSmallPose`,
`AvbdContact.WorldCollideCylinderRimRowsPersistAcrossSmallPose`,
`AvbdContact.WorldCollideCapsuleCapRowsPersistAcrossSmallPose`, and
`AvbdContact.WorldCollideCapsuleSideRowsPersistAcrossSmallPose`. They replay
live cap/side/rim contacts under small same-feature nudges and verify stable
endpoint-key row ordinals, primitive feature IDs, Coulomb bounds, warm-started
normal lambdas, and projected friction duals where the tangent basis rotates.
The focused `AvbdContact.WorldCollide*` filter passed 31/31 after lint. This
closes only
cylinder/capsule small-pose persistence and no broad pile/contact persistence,
CPU-win, GPU, or paper-number gate.

Latest local follow-up: mesh-face and mesh-edge private rigid-contact
small-pose persistence now have
`AvbdContact.WorldCollideMeshFaceRowsPersistAcrossSmallPose` and
`AvbdContact.WorldCollideMeshEdgeRowsPersistAcrossSmallPose`. They replay live
sphere/mesh contacts on the same triangle face or edge, nudge the contact
locations while preserving the feature, and verify stable endpoint-key row
ordinals, feature IDs, Coulomb bounds, warm-started normal lambdas, and
projected world-space friction duals. The focused two-test filter passed
locally. This closes only mesh face/edge small-pose persistence and no broad
pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: mesh-vertex private rigid-contact small-pose
persistence now has
`AvbdContact.WorldCollideMeshVertexRowsPersistAcrossSmallPose`. It keeps live
sphere/mesh contacts active at all three triangle vertex endpoint features,
nudges the sphere poses within the same vertex regions, and verifies stable
endpoint-key rows, vertex feature local indices, Coulomb bounds, warm-started
normal lambdas, and projected world-space friction duals. The focused
`AvbdContact.WorldCollide*` filter passed 24/24 locally. This closes only a
mesh-vertex small-pose persistence slice and no broad pile/contact
persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: mesh-vertex private rigid-contact replay coverage now
has `AvbdContact.WorldCollideMeshVertexRowsIgnoreContactOrder`. It replays live
sphere/mesh contacts at all three triangle vertex endpoint features in reverse
order, verifying vertex feature local indices, stable endpoint-key rows,
Coulomb bounds, and warm-started normal/friction lambdas. The focused
`AvbdContact.WorldCollide*` filter passed 23/23 locally. This closes only a
mesh-vertex replay slice and no broad
pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: private rigid-contact box-pile contact-order coverage
now has
`AvbdContact.WorldCollideBoxPileFrictionRowsIgnoreContactOrder`. It reuses the
wider two-lower-box plus one-spanning-top pile, reverses the live
`World::collide()` contact vector, and verifies row ordinals, per-pair friction
coefficients, Coulomb bounds, and warm-started normal/friction lambdas survive
raw contact-order changes. The focused `AvbdContact.WorldCollide*` filter
passed 22/22 after lint. This closes only a private box-pile contact-order
slice and no broad pile/contact persistence, CPU-win, GPU, or paper-number
gate.

Latest local follow-up: live contact-stage box-pile friction coverage now has
explicit-stage and default-schedule regressions,
`World.RigidBodyContactStageAvbdWarmStartedBoxPileManifoldFrictionReducesSlip`
and
`World.RigidBodyContactStageAvbdWarmStartedBoxPileManifoldFrictionRunsThroughDefaultWorldStep`.
They reuse the wider two-lower-box plus one-spanning-top pile, configure all
dynamic bodies for private AVBD contact, and verify warm-started friction
reduces top/lower tangential slip through both the manual `RigidBodyContactStage`
pipeline and built-in `World::step()` schedule. The focused
`World.RigidBodyContactStageAvbd*` filter passed 42/42 after lint. This closes
only a narrow live box-pile contact-stage slice and no broad pile/contact
persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: private rigid-contact endpoint-order coverage now also
has a wider live box-pile friction regression,
`AvbdContact.WorldCollideBoxPileFrictionRowsIgnoreEndpointOrder`. It reuses
the two-lower-box plus one-spanning-top pile, swaps every raw contact endpoint,
and verifies that per-pair Coulomb bounds persist while paired tangent duals
project into the swapped physical tangent basis. The focused test and
`AvbdContact.WorldCollide*` filter passed locally. This closes only a private
box-pile endpoint-order slice and no broad pile/contact persistence, CPU-win,
GPU, or paper-number gate.

Latest local follow-up: private rigid-contact row-persistence coverage now has
a wider live pile-style regression,
`AvbdContact.WorldCollideBoxPileFrictionRowsPersistAcrossSmallPose`. The test
uses two lower dynamic boxes on a static ground with one wider top body spanning
both supports, then verifies that per-pair row ordinals, friction coefficients,
Coulomb bounds, and warm-started friction dual state persist across a small
pose nudge. The focused test passed locally. This closes only a private
box-pile row-persistence slice and no broad pile/contact persistence, CPU-win,
GPU, or paper-number gate.

Latest local follow-up: default `World::step()` contact-stage AVBD coverage now
also reaches the multi-top stacked box-manifold friction path through
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`.
The test keeps a static ground, one lower dynamic box, and two independently
sliding upper dynamic boxes active with normal load, then checks that friction
reduces both upper/lower dynamic contact-point slips against a frictionless
baseline. The focused test passed locally. This closes only the default-step
schedule evidence for that narrow multi-contact stacked friction slice and no
broad pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now also includes a
multi-top stacked box-manifold friction regression,
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip`.
The test configures a static ground, one supported lower dynamic box, and two
independently sliding upper dynamic boxes in one live contact set, then checks
that warm-started friction reduces both upper/lower dynamic contact-point slips
against a frictionless baseline. The focused test passed locally. This closes
only a narrow multi-contact stacked contact-stage friction slice and no
default-step pile, broad pile/contact persistence, CPU-win, GPU, or
paper-number gate.

Latest local follow-up: default `World::step()` contact-stage AVBD coverage now
also reaches the stacked static/dynamic plus dynamic/dynamic box-manifold
warm-started friction path through
`World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`.
The test keeps the stacked manifolds active with normal load, then checks that
friction reduces upper/lower contact-point slip against a frictionless
baseline. The focused test passed locally. This closes only the default-step
schedule evidence for the stacked contact-stage friction slice and no broad
pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now includes a stacked
static/dynamic plus dynamic/dynamic box-manifold warm-started friction
regression,
`World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionReducesSlip`.
The test configures a static ground, lower dynamic box, and sliding upper
dynamic box in one live contact set, then checks that friction reduces the
upper/lower contact-point slip against a frictionless baseline. The focused
test passed locally. This closes only the stacked contact-stage friction slice
and no broad pile/contact persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD now also keeps default
`World::step()` dynamic/dynamic box-box manifold friction active when a moving
manifold has active penetration but no lagged normal force yet, using a
depth-based normal-force fallback for Coulomb row bounds. The new
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep`
regression passed, and the full `World.RigidBodyContactStageAvbd*` focused
filter now passes 36 tests locally. This closes only the default-step
dynamic/dynamic box-manifold friction slice and no CPU-win, GPU, or
paper-number gate.

Latest local follow-up: dartpy explicit-anchor one-DOF articulated motor
break/skip/reset coverage now also asserts endpoint and axis shape after reset
across same-multibody and world-link revolute/prismatic cases, including the
non-cardinal same-multibody pair and world-link reset regressions. The focused
selected pytest filter over those five articulated motor reset tests passed
locally after rebuilding dartpy. This closes only the endpoint/axis-shape
assertion slice and no broad articulated motor lifecycle, fracture corpus,
CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now also includes live
dynamic/dynamic box-box manifold warm-started friction through
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`.
The focused dynamic/dynamic box-manifold filter passed locally; the full
`World.RigidBodyContactStageAvbd*` focused filter now passes 35 tests locally.
This closes only the stage-level dynamic/dynamic box-manifold friction slice
and no default-step persistence, CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now also includes live
box-box manifold warm-started friction through
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`
and
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`.
The full `World.RigidBodyContactStageAvbd*` focused filter now passes 34 tests
locally; this closes only the narrow box-manifold static/dynamic friction slice
and no CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now also includes
simultaneous multi-contact warm-started friction through
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`
and
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`.
The full `World.RigidBodyContactStageAvbd*` focused filter now passes 32
tests locally; this closes only the simultaneous configured static/dynamic
friction slice and no CPU-win, GPU, or paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now also includes
enabled-peer/disabled-peer warm-started friction through
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`
and
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`.
The full `World.RigidBodyContactStageAvbd*` focused filter now passes 30
tests locally; this closes only the paired-config static/dynamic friction slice
and no CPU-win, GPU, or paper-number gate.

Latest local follow-up: after the requested latest `origin/main` merge, the
branch is based on `03c306e6ca4` ("Plan Simplicits and PPF solver intake
(#2952)") and the staged AVBD batch was restored without conflicts. The project
`.gitignore` now ignores scratch JSON dumps under the PLAN-104 AVBD evidence
folder while allowing curated `avbd-*-packet.json` fixtures that are referenced
by docs and packet-writer tests. Contact-stage AVBD coverage now also includes
kinematic-owned warm-started friction through
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`
and
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`.
The full `World.RigidBodyContactStageAvbd*` focused filter passed locally; this
closes only the
kinematic-owned static/dynamic friction slice and no CPU-win, GPU, or
paper-number gate.

Latest local follow-up: contact-stage AVBD coverage now includes the first live
both-configured dynamic/dynamic, single-config dynamic/dynamic, dynamic-owned
static/dynamic, default-step both-configured dynamic/dynamic,
default-step single-config dynamic/dynamic, contact-to-position pipeline
integration, default
`World::step()` integration, static-owned static/dynamic,
default-step static-owned static/dynamic,
stored-static-velocity ignore, kinematic-as-prescribed endpoint,
default-step kinematic-prescribed endpoint, enabled-peer/disabled-peer,
default-step enabled-peer/disabled-peer, configured multi-contact,
default-step configured multi-contact, mixed-config,
default-step mixed-config,
disabled-config, default-step disabled-config, static/dynamic friction,
static-owned static/dynamic friction, default-step static/dynamic friction,
kinematic-owned static/dynamic friction, default-step static-owned
static/dynamic friction, default-step kinematic-owned static/dynamic friction,
enabled-peer/disabled-peer static/dynamic friction, default-step
enabled-peer/disabled-peer static/dynamic friction, simultaneous
multi-contact static/dynamic friction, default-step simultaneous multi-contact
static/dynamic friction, box-manifold static/dynamic friction,
dynamic/dynamic box-manifold friction, default-step box-manifold
static/dynamic friction, default-step dynamic/dynamic box-manifold friction,
dynamic/dynamic friction, and default-step dynamic/dynamic friction rigid-body
contact-stage regressions.
The new
`World.RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity` case
configures two overlapping dynamic free rigid bodies with the private
`RigidAvbdContactConfig`, and the new
`World.RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig` case
checks the same dynamic/dynamic activation rule when only one dynamic endpoint
is configured. The
new `World.RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep`
case checks that the both-configured dynamic/dynamic projection feeds through
the built-in `World::step()` schedule, and the new
`World.RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep`
case checks the same default schedule path when only one dynamic endpoint is
configured. The
`World.RigidBodyContactStageAvbdProjectsStaticDynamicContactVelocity` checks a
static ground/dynamic sphere contact where only the dynamic body opts in. The
new `World.RigidBodyContactStageAvbdFeedsRigidBodyPositionStage` case checks
that the projected velocity advances the dynamic sphere when
`RigidBodyPositionStage` follows the contact stage in the same pipeline. The
new `World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep` case checks
the same AVBD contact projection through the built-in `World::step()` schedule.
The
new `World.RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig` case
checks the opposite static-owned opt-in path where the static ground carries the
private config. The new
`World.RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep` case
checks the same static-owned opt-in path through the built-in `World::step()`
schedule. The new
`World.RigidBodyContactStageAvbdIgnoresStoredStaticVelocity` case checks that a
stored static-body velocity does not perturb the static-owned AVBD projection
compared with an otherwise identical zero-static-velocity baseline. The new
`World.RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle` case
checks that kinematic endpoints stay fixed in the AVBD snapshot and that their
prescribed velocity does not perturb the dynamic body's projected contact-stage
velocity compared with a zero-kinematic-velocity baseline. The new
`World.RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep` case
checks the same prescribed-endpoint behavior through the built-in
`World::step()` schedule and verifies the projected velocity feeds the rigid
position stage while the kinematic endpoint remains fixed. The new
`World.RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig` case
checks that an explicitly disabled peer config does not veto AVBD activation
when the other contact endpoint carries an enabled private config. The new
`World.RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep`
case checks the same enabled-peer/disabled-peer activation through the built-in
`World::step()` schedule. The new
`World.RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts` case
checks two configured dynamic spheres against one static ground in the same live
contact set. The new
`World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep`
case checks the same configured multi-contact projection through the built-in
`World::step()` schedule. The new
`World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet` case checks
that a contact set containing any unconfigured contact falls back as a whole
instead of partially routing configured contacts through AVBD. The new
`World.RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep`
case checks the same all-or-nothing fallback through the built-in
`World::step()` schedule. The new
`World.RigidBodyContactStageAvbdDisabledConfigFallsBack` case checks that
`RigidAvbdContactConfig::enabled = false` opts out to the ordinary rigid contact
response. The new
`World.RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep`
case checks the same disabled opt-out through the built-in `World::step()`
schedule. The new
`World.RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide` case enables
retained normal-dual warm starting and verifies live contact-stage friction
tangent rows reduce static/dynamic contact-point slip against a frictionless
baseline. The new
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide`
case verifies the same stage-only warm-started friction behavior when the
private opt-in config is attached to the static endpoint. The new
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`
case verifies the adjacent kinematic-owned warm-started friction path and
checks that a prescribed kinematic tangential velocity is preserved but ignored
as contact slip input. The new
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`
case verifies that a disabled peer config does not veto warm-started friction
rows when the other endpoint carries an enabled private config. The new
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`
case verifies simultaneous configured static/dynamic contacts both feed
warm-started friction rows in one contact-stage solve. The new
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`
case verifies that live box-box manifolds with multiple narrow-phase contact
points feed warm-started contact-stage friction rows. The new
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`
case verifies that a live dynamic/dynamic box-box manifold feeds the same
warm-started contact-stage friction rows and reduces relative contact-point
slip against a frictionless baseline. The new
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip` case
uses the same retained normal-dual warm start to verify dynamic/dynamic sphere
contacts reduce relative tangential contact-point slip. The new
`World.RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep`
case verifies the built-in `World::step()` schedule reaches the same
warm-started static/dynamic friction path, and the new
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep`
case verifies the built-in schedule reaches the static-owned warm-started
static/dynamic friction path. The new
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`
case verifies the built-in schedule reaches the kinematic-owned warm-started
static/dynamic friction path without using the prescribed kinematic tangential
velocity as slip input. The new
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`
case verifies the built-in schedule reaches the paired enabled-peer/disabled-
peer warm-started static/dynamic friction path. The new
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`
case verifies the built-in schedule reaches simultaneous configured
static/dynamic warm-started friction contacts. The new
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`
case verifies the built-in schedule reaches live box-box manifold
warm-started friction contacts. The new
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep`
case does the same for dynamic/dynamic sphere contacts. The stage-only cases
verify separating velocity projection without direct pose moves, the pipeline
case verifies the velocity feeds the following rigid position stage, the
default-step cases verify the built-in schedule reaches the same
static/dynamic, kinematic, enabled-peer/disabled-peer, configured
multi-contact, box-manifold, fallback, disabled-config, and friction paths, the
static-velocity case verifies static-body stored velocities are ignored by the
static-owned AVBD projection, and the friction cases verify the lagged
normal-force contact-stage friction path.
Focused validation built and ran the C++ world AVBD
contact-stage filter. This closes only narrow contact-stage dynamic-contact
activation/fallback/friction evidence slices; broader contact-stage coverage,
CPU-win, GPU, and paper-number gates remain open.

Latest local follow-up: dartpy now exposes `World.save_binary(path)` and
`World.load_binary(path)` for the existing simulation binary serializer. The
new `test_simulation_world_articulated_binary_roundtrip_from_python` and
`test_simulation_world_articulated_binary_roundtrip_from_python_completes_one_dof_endpoint_types`
cases round-trip all same-multibody/world-link public articulated
revolute/prismatic broken motor rows through Python, verify endpoint/type, axis,
command, and effort-limit state after load, and prove reset rebuilds the hard
rows plus reversed free-axis motor motion. The companion
`test_simulation_world_articulated_fixed_spherical_binary_roundtrip_from_python`
and
`test_simulation_world_articulated_fixed_spherical_binary_roundtrip_from_python_completes_endpoint_types`
cases now round-trip all same-multibody/world-link fixed/spherical broken rows,
verify restored endpoint/type shape, and prove reset rebuilds fixed all-axis
rows and spherical linear rows while leaving spherical orientation free. The
additional `test_simulation_world_articulated_design_binary_rebuild_from_python`
and
`test_simulation_world_articulated_design_binary_rebuild_from_python_completes_fixed_spherical_endpoint_types`
cases save all same-multibody/world-link fixed/spherical/revolute/prismatic
public facades before simulation mode, reload them through dartpy, and prove
simulation entry rebuilds the private all-axis fixed rows, one-DOF hard rows
plus free-axis motor rows, and spherical linear rows while leaving spherical
orientation free. Focused validation rebuilt dartpy and ran the Python binary
roundtrip/rebuild regressions plus the API-name check.
This closes only one dartpy public-facade persistence evidence slice; it closes
no CPU-win, GPU, or paper-number gate.

Latest local follow-up: finite-effort articulated motor projection now gives
hard anchor/axis rows priority after clamping bounded free-axis motor impulses,
so a saturated tiny effort cap cannot leak into locked off-origin coordinates.
The broadened
`test_simulation_world_articulated_non_cardinal_off_origin_tiny_effort_motors_from_python`
case verifies the Python public facades preserve tiny effort-limit arrays,
normalize non-cardinal axes, keep off-origin hard rows bounded, and keep
free-axis motion far below unbounded velocity targets for all four public
one-DOF endpoint/type combinations. Focused validation also rebuilt dartpy and
reran the broadened direct C++ tiny-limit filter for same-multibody,
movable-pair, and world-link off-origin anchors. This closes only one
public-facade finite-effort cap evidence slice; it closes no CPU-win, GPU, or
paper-number gate.

Latest local follow-up: dartpy same-multibody articulated revolute/prismatic
motor break/reset coverage now also includes non-cardinal explicit-anchor axes.
The new
`test_simulation_world_articulated_non_cardinal_pair_motors_reset_from_python`
case verifies the Python public facades normalize both axes, break each motor,
allow constrained anchor/axis drift while skipped, and re-engage the
non-cardinal same-multibody one-DOF rows after reset with reversed velocity
commands. This closes only one Python public-facade articulated pair lifecycle
evidence slice; it closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: dartpy world-link revolute motor break/reset coverage
now also includes a non-cardinal explicit-anchor axis. The new
`test_simulation_world_articulated_non_cardinal_revolute_reset_from_python`
case verifies the Python public facade normalizes the axis, breaks the motor,
allows anchor drift while skipped, and re-engages the non-cardinal revolute
rows after reset with a reversed velocity command. This closes only one Python
public-facade articulated one-DOF lifecycle evidence slice; it closes no
CPU-win, GPU, or paper-number gate.

Latest local follow-up: dartpy world-link prismatic motor break/reset coverage
now includes a non-cardinal explicit-anchor axis. The new
`test_simulation_world_articulated_non_cardinal_prismatic_reset_from_python`
case verifies the Python public facade normalizes the axis, breaks the motor,
allows orthogonal drift while skipped, and re-engages the non-cardinal
prismatic rows after reset with reversed velocity command. This closes only one
Python public-facade articulated one-DOF lifecycle evidence slice; it closes no
CPU-win, GPU, or paper-number gate.

Latest local follow-up: live mesh-vertex endpoint feature coverage now verifies
that an actual sphere/triangle-vertex `World::collide()` contact carries the
mesh vertex shape-local point into AVBD feature encoding. The new
`AvbdContact.PlaneAndMeshFeatureKeysUseNarrowPhaseShapeLocalPoints` block
checks vertex feature kind/index 0 for the contacted mesh endpoint instead of
leaving vertex coverage to the geometric encoder alone. Focused validation
passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.PlaneAndMeshFeatureKeysUseNarrowPhaseShapeLocalPoints'`.
This closes only one mesh-vertex endpoint-feature evidence slice; it closes no
CPU-win, GPU, or paper-number gate.

Latest local follow-up: live mesh-edge row-order coverage now replays actual
sphere/triangle-edge `World::collide()` contacts at multiple positions on the
same mesh edge. `AvbdContact.WorldCollideMeshEdgeRowsIgnoreContactOrder`
checks the mesh endpoint is encoded as the packed vertex-pair edge feature and
that reversing contact order keeps deterministic canonical endpoint-pair row
ordinals. The existing mesh-face replay helper now also pins the mesh face
local feature ID instead of only checking row order. Focused validation passed
`pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideMeshFaceRowsIgnoreContactOrder:AvbdContact.WorldCollideMeshEdgeRowsIgnoreContactOrder:AvbdContact.WorldCollide*'`.
This closes only one mesh edge/face feature replay evidence slice; it closes no
CPU-win, GPU, or paper-number gate.

Latest local follow-up: live cylinder-cap and capsule-cap row-order coverage
now verifies both positive and negative cap feature codes. The tests replay
real primitive/plane `World::collide()` contacts against upward- and
downward-facing planes and keep deterministic canonical endpoint-pair row
ordinals when replayed in reverse contact order.
`AvbdContact.WorldCollideCylinderCapRowsIgnoreContactOrder` and
`AvbdContact.WorldCollideCapsuleCapRowsIgnoreContactOrder` now pin the cap
local feature IDs instead of checking only generic face kind. Focused validation
passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideCylinderCapRowsIgnoreContactOrder:AvbdContact.WorldCollideCapsuleCapRowsIgnoreContactOrder:AvbdContact.WorldCollide*'`.
This closes only one curved cap primitive row-order evidence slice; it closes
no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live cylinder-rim row-order coverage now verifies that
real sphere/cylinder `World::collide()` contacts on the top and bottom rims
encode the cylinder endpoint as positive/negative edge features and keep
deterministic canonical endpoint-pair row ordinals when replayed in reverse
contact order.
`AvbdContact.WorldCollideCylinderRimRowsIgnoreContactOrder` extends the live
curved-primitive coverage beyond cap and side face patches. Focused validation
passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideCylinderRimRowsIgnoreContactOrder:AvbdContact.WorldCollide*'`.
This closes only one curved rim/edge primitive row-order evidence slice; it
closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live cylinder-side and capsule-side row-order coverage
now verifies that real sphere/primitive `World::collide()` contacts on curved
side patches keep the same canonical endpoint-pair row ordinals when replayed
in reverse contact order. `AvbdContact.WorldCollideCylinderSideRowsIgnoreContactOrder`
and `AvbdContact.WorldCollideCapsuleSideRowsIgnoreContactOrder` also check the
side-patch feature codes rather than the already covered cap features. Focused
validation passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideCylinderSideRowsIgnoreContactOrder:AvbdContact.WorldCollideCapsuleSideRowsIgnoreContactOrder:AvbdContact.WorldCollide*'`.
This closes only one curved-side primitive row-order evidence slice; it closes
no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live sphere/plane friction tangent projection now
verifies that when a real `World::collide()` plane normal changes, persisted
paired tangent-row dual state is projected as a physical world tangent impulse
into the new contact tangent basis instead of being reused as basis-local
scalars. `AvbdContact.WorldCollideFrictionRowsProjectAcrossChangingPlaneNormal`
also checks stable canonical endpoint keys, stable row ordinals, friction
coefficient combination, Coulomb bounds from the persisted normal row, and the
generated friction-row state. Focused validation passed
`pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideFrictionRowsProjectAcrossChangingPlaneNormal:AvbdContact.WorldCollide*'`.
This closes only one live normal-rotation rigid-contact friction evidence
slice; it closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live stacked endpoint-swapped friction coverage now
verifies that simultaneous static/dynamic and dynamic/dynamic box manifolds
preserve paired tangent-row warm-start state as a physical world tangent dual
when every raw `World::collide()` contact arrives with body endpoints, shape
indices, local points, and normal polarity swapped.
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreEndpointOrder` also
checks multiple canonical face-pair row groups, distinct per-pair friction
coefficients, and per-pair Coulomb bounds. Focused validation passed
`pixi run build-simulation-tests`,
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreEndpointOrder:AvbdContact.WorldCollide*'`,
and `pixi run lint`. This closes only one stacked endpoint-order friction-row
persistence evidence slice; it closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live endpoint-swapped box-box manifold friction
coverage now verifies that paired tangent-row warm-start state projects as a
physical world tangent dual when every raw `World::collide()` contact arrives
with body endpoints, shape indices, local points, and normal polarity swapped.
`AvbdContact.WorldCollideLiveManifoldFrictionRowsIgnoreEndpointOrder` also
checks the same canonical face-pair row groups and Coulomb bounds as the
normal-row endpoint-order coverage. Focused validation passed
`pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollideLiveManifoldFrictionRowsIgnoreEndpointOrder:AvbdContact.WorldCollide*'`.
This closes only one endpoint-order friction-row persistence evidence slice; it
closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live stacked-box manifold friction tangent warm-start
coverage now verifies that simultaneous static/dynamic and dynamic/dynamic box
manifolds retain paired tangent-row dual state and per-pair Coulomb bounds by
endpoint/feature row key across a small stack nudge through
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsPersistAcrossSmallPose`.
Focused validation passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollide*'`, plus
`pixi run lint`. This closes only one stacked rigid-contact friction-row
persistence evidence slice; it closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live box-box manifold friction tangent warm-start
coverage now verifies that a small pose-preserving box-box nudge retains
paired tangent-row dual state and Coulomb bounds by the same endpoint/feature
row keys through
`AvbdContact.WorldCollideLiveManifoldFrictionRowsPersistAcrossSmallPose`.
Focused validation passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollide*'`, plus
`pixi run lint`. This closes only one live box-box friction-row persistence
evidence slice; it closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: live sphere/plane friction tangent warm-start coverage
now verifies that reversing actual `World::collide()` contact order preserves
paired tangent-row dual state and Coulomb bounds by row key through
`AvbdContact.WorldCollideFrictionRowsIgnoreContactOrder`. Live
sphere/mesh-face row-order coverage now replays
actual mesh-face contacts at multiple positions and verifies that reversing
the contact input does not move same-feature row ordinals through
`AvbdContact.WorldCollideMeshFaceRowsIgnoreContactOrder`. Live box-box
manifold row-persistence coverage now also
nudges one box by a small amount along the contact normal and verifies that the
same canonical face-pair rows retain their row ordinals and in-face local
coordinates through
`AvbdContact.WorldCollideLiveManifoldSameFeatureRowsPersistAcrossSmallPose`.
Curved primitive row-order coverage now replays actual cylinder-cap/plane and
capsule-cap/plane contacts at multiple positions and verifies that reversing
the contact input does not move same-feature row ordinals through
`AvbdContact.WorldCollideCylinderCapRowsIgnoreContactOrder` and
`AvbdContact.WorldCollideCapsuleCapRowsIgnoreContactOrder`.
Live stacked box coverage now also verifies that simultaneous static/dynamic
and dynamic/dynamic box manifolds preserve canonical endpoint-pair row ordinals
and in-plane local coordinates across a small pose-preserving nudge through
`AvbdContact.WorldCollideStackedManifoldsPersistRowsAcrossSmallPose`. Focused
validation passed `pixi run build-simulation-tests` and
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollide*'` plus
`pixi run lint`. This closes only the curved-primitive row-order and small-pose
live rigid-contact row-persistence evidence slices; it closes no CPU-win, GPU,
or paper-number gate.

Latest local follow-up: after the requested latest `origin/main` merge, the
branch is based on `556b894fb5f` ("Share Phase 3 benchmark and CCD diagnostics
contracts (#2948)") and the staged AVBD batch was restored with
`git stash apply --index stash@{0}` without conflicts. Live `World::collide()`
endpoint-order regressions now prove that swapping contact body endpoints,
shape indices, local points, and normal polarity preserves the canonical
endpoint row key and row ordinal for both sphere/plane contact points and the
multi-point box-box manifold. Focused validation passed
`pixi run build-simulation-tests`,
`test_boxed_lcp_contact --gtest_filter='AvbdContact.WorldCollide*'`, and
`pixi run lint`. This closes only the live endpoint-order row-identity evidence
slice; it closes no CPU-win, GPU, or paper-number gate.

Latest local follow-up: the branch was fast-forwarded to live `origin/main`
at `0adb97229ff` using an HTTPS fetch after SSH fetch failed with
`Network is unreachable`, and the staged AVBD batch was restored with
`git stash apply --index` without conflicts. The merge backup remains at
`stash@{0}` and should not be dropped without maintainer approval. Post-merge
focused validation passed `pixi run build-simulation-tests`, rebuilt
`bm_avbd_rigid_fixed_joint`, passed the focused C++ ctest set
(`test_variational_integration`, `test_boxed_lcp_contact`,
`test_serialization`, `test_world`, and `test_world_step_profiling`), and
passed the 13 focused packet-writer tests for the refreshed 2D/3D Spring and
3D Soft Body rows. A 2D Soft Body endpoint-uniqueness row-counter fast path was
implemented experimentally, rebuilt, and then reverted because packet-style
benchmark smoke did not support keeping it. This closes no CPU-win, GPU, or
paper-number gate.

Latest local follow-up: the public articulated revolute/prismatic motor
py-demos now opt into the shared replay panel as stateless world-time-driven
controllers, and the high-ratio articulated-chain py-demo now captures and
restores its replay-loop UI state while syncing the render bridge. This fixed
the `test_registered_world_scenes_receive_shared_replay_controls` failure found
by `pixi run test-py`; the focused replay-panel regression and the full Python
gate now pass. This is demo/replay validation coverage only and closes no
source-row CPU-win, GPU, or paper-number gate.

Latest local follow-up: private direct world-link AVBD revolute/prismatic
point-joint configs now serialize their `AvbdRigidWorldPointJointConfig` rows
through binary save/load, including anchors, masks, bases, stiffness fields, and
free-axis velocity motor state. Focused C++ coverage in
`VariationalIntegration.AvbdBreakableRevoluteVelocityPointJointConfigSurvivesSaveLoadAndReset`
and
`VariationalIntegration.AvbdBreakablePrismaticVelocityPointJointConfigSurvivesSaveLoadAndReset`
reloads broken private rows, verifies the restored config, proves broken rows
stay skipped under external force, then resets and re-engages the hard rows plus
updated motor command. While validating this, a kinematics-cache regression from
the clean-frame graph shortcut was fixed by marking multibody link frame caches
dirty after semi-implicit and variational multibody position writes. Focused
validation rebuilt `test_variational_integration`, reran the previously stale
articulated motor filters plus `ExternalForce.*`, and reran the two new
save/load/reset regressions. This is still a narrow private lifecycle and
cache-correctness slice; it closes no broad paper-parity, GPU, or source-row
benchmark gate.

Latest local follow-up: rigid AVBD finite-stiffness point-pair, angular-pair,
and distance-spring row updates now return immediately once the effective
stiffness is already at the material cap, clearing the finite-row dual without
recomputing the current constraint geometry. This preserves the existing update
result for capped public rigid distance springs and capped compliant
point-joint rows. Focused coverage now includes
`AvbdRigidBlock.PointPairFiniteUpdateKeepsCappedStiffness`,
`AvbdRigidBlock.RigidAngularFiniteUpdateKeepsCappedStiffness`, and
`AvbdRigidBlock.PointPairDistanceSpringUpdateKeepsCappedStiffness`. Focused
validation rebuilt `dart_simulation_tests`, ran the full 87-test
`test_avbd_rigid_block` binary, rebuilt `bm_avbd_rigid_fixed_joint`, and
smoke-ran selected Rod/Joint Grid/Soft Body/Spring rows. The broader diagnostic
ran under load average around 11.7, so it is capped-row overhead smoke only; no
refreshed packet CPU-win, GPU, or paper-number gate is closed.

Latest local follow-up: rigid AVBD friction tangent-pair evaluation now detects
the common contact-row case where the two tangent rows share the same body-local
anchors, then reuses the first row's transformed world anchors for the second
row in the generic force helper, direct block assembly, and row-state update
path. Hand-built rows with distinct anchors still take the existing per-row
transform path. Focused validation rebuilt `dart_simulation_tests`, ran the
full 84-test `test_avbd_rigid_block` binary, and smoke-ran
`BM_AvbdDemo2dDynamicFrictionStep`, `BM_AvbdDemo2dStaticFrictionStep`,
`BM_AvbdDemo2dCardsStep`, and `BM_AvbdDemo2dNetStep` with
`--benchmark_min_time=0.1s`. The diagnostic smoke ran under load average around
4.3, recording about 22.9 us Dynamic Friction, 5.99 us Static Friction,
0.507 ms Cards, and 0.433 ms Net CPU time per step. This is mechanical
hot-path cleanup evidence only; no refreshed packet CPU-win, GPU, or
paper-number gate is closed.

Latest local follow-up: rigid AVBD friction tangent-pair row-state updates now
reuse the regularized 2D tangent constraint vector that is already needed for
stiffness growth when computing the cone-projected friction force, instead of
calling the older force helper and recomputing the same tangent constraints.
Focused validation rebuilt `dart_simulation_tests`, ran the full 84-test
`test_avbd_rigid_block` binary, and smoke-ran
`BM_AvbdDemo2dDynamicFrictionStep`, `BM_AvbdDemo2dStaticFrictionStep`, and
`BM_AvbdDemo2dCardsStep` with `--benchmark_min_time=0.1s`. The diagnostic
smoke ran under load average around 7.5, recording about 14.2 us Dynamic
Friction, 5.51 us Static Friction, and 0.472 ms Cards CPU time per step. This
is still local overhead-cleanup evidence only; no refreshed packet CPU-win,
GPU, or paper-number gate is closed.

Latest local follow-up: rigid AVBD friction tangent-pair assembly now computes
each tangent row's world anchors once, feeds the precomputed constraint values
through the shared static/dynamic friction force helper, and stamps both body
directions from those same world points. This preserves the existing friction
cone semantics while avoiding repeated local-to-world point transforms inside a
single tangent-pair evaluation. Focused validation rebuilt
`dart_simulation_tests`, ran the full 84-test `test_avbd_rigid_block` binary,
and smoke-ran `BM_AvbdDemo2dCardsStep`, `BM_AvbdDemo2dJointGridStep`,
`BM_AvbdDemo2dSoftBodyStep`, `BM_AvbdDemo2dRopeStep`, and
`BM_AvbdDemo2dNetStep` with `--benchmark_min_time=0.1s`. The diagnostic smoke
ran under load average around 6.9 and low reported CPU frequency, recording
about 0.550 ms Cards, 10.7 ms Joint Grid, 2.43 ms Soft Body, 0.135 ms Rope, and
0.442 ms Net CPU time per step. This is still overhead-cleanup evidence only;
no refreshed packet CPU-win, GPU, or paper-number gate is closed.

Latest local follow-up: after fetching `origin/main` through a one-shot HTTPS
transport override, the requested merge was a no-op (`Already up to date`).
Rigid AVBD point-pair and distance-spring helpers now carry the world anchors
they already compute for constraint values through direct force directions and
the distance-spring Hessian/Jacobian path. The local-point Hessian API remains
available for direct callers, while the block assembly and direct spring helper
avoid recomputing the same local-to-world point just to build a point Jacobian.
Focused validation rebuilt `dart_simulation_tests`, ran the full 84-test
`test_avbd_rigid_block` binary, and smoke-ran
`BM_AvbdDemo2dJointGridStep`, `BM_AvbdDemo2dSoftBodyStep`,
`BM_AvbdDemo2dRopeStep`, and `BM_AvbdDemo2dNetStep` with
`--benchmark_min_time=0.1s`. The second diagnostic smoke, under load average
around 5.5 and low reported CPU frequency, recorded about 10.1 ms Joint Grid,
3.78 ms Soft Body, 0.207 ms Rope, and 0.359 ms Net CPU time per step, so this
is overhead-cleanup evidence only; no refreshed packet CPU-win, GPU, or
paper-number gate is closed.

Latest local follow-up: the rigid AVBD block row assembly now reuses the
per-row world anchor points, relative vector, and spring axis that it already
computed for point-pair and distance-spring constraint values when stamping the
same body's force/Hessian contribution. This avoids redundant local-to-world
point transforms in the contact-stage projection loop for rope, net,
Joint Grid, and soft-body source rows without changing row identities or solver
semantics. The same block loop now also reuses the shared angular orientation
error across consecutive all-axis angular rows, avoiding three identical
quaternion error evaluations per fixed-joint angular triplet during block
assembly and row-state updates. Focused validation rebuilt
`dart_simulation_tests`, ran the full 84-test `test_avbd_rigid_block` binary,
smoke-ran `BM_AvbdDemo2dJointGridStep`, `BM_AvbdDemo2dSoftBodyStep`,
`BM_AvbdDemo2dRopeStep`, and `BM_AvbdDemo2dNetStep` with
`--benchmark_min_time=0.1s`, and ran the matching four Python source-row cycle
tests. The smoke reported about 10.1 ms Joint Grid, 2.14 ms Soft Body,
0.120 ms Rope, and 0.389 ms Net CPU time per step on this host, but this is
still non-packet evidence; no CPU-win, GPU, or paper-number gate is closed
without refreshed same-command packets.

Latest local follow-up: reserved AVBD rigid-world body-index maps now become
active immediately once a large source-row snapshot requests hash lookup
capacity, instead of leaving the first 16 bodies on the linear lookup path
before the map is populated. Small snapshots still keep the no-map fast path,
with focused coverage in
`AvbdRigidBlock.RigidWorldAppendRowsUseLinearBodyIndexForSmallSnapshots` and
the new
`AvbdRigidBlock.RigidWorldAppendRowsUseReservedBodyIndexMapImmediately`
regression. While validating the affected public source rows, the Hanging Rope
source-row invariant was adjusted to wait through the heavy-block settling
transient before asserting lift and endpoint contraction. Focused validation
rebuilt `dart_simulation_tests`, ran the new filters, ran the full 84-test
`test_avbd_rigid_block` binary, rebuilt `bm_avbd_rigid_fixed_joint`, smoke-ran
the Joint Grid, Soft Body, Rope, Heavy Rope, Hanging Rope, and Net 2D benchmark
rows under high load, and ran the matching six Python source-row invariants.
This is a source-row lookup overhead and invariant-hardening cleanup only; no
CPU-win, GPU, or paper-number gate is closed without refreshed same-command
packets.

Latest local follow-up: `KinematicsStage::prepare()` now reuses its cached
`WorldKinematicsGraph` until the world's frame-topology revision changes,
instead of rebuilding the graph on every prepared step. The stale-topology path
still rebuilds through `WorldKinematicsGraph::isTopologyCurrent()`, while
unchanged source rows avoid a no-op prepare-time graph rebuild before the
existing clean-frame-cache execution skip. Focused validation rebuilt
`dart_simulation_tests`, ran the clean/stale kinematics graph regressions, the
full `test_world_step_profiling` binary, and the full 83-test
`test_avbd_rigid_block` binary. The `bm_avbd_rigid_fixed_joint` target also
rebuilt, and a short diagnostic smoke over the Motor, Rope, Hanging Rope,
Spring, and Spring Ratio 2D source rows reported about 10.1 us, 160 us, 334 us,
4.6 us, and 40.5 us per step respectively. Python step profiling for those
rows still shows rigid-body contact as the dominant stage, so this is a narrow
kinematics-prepare overhead cleanup only. It closes no source-row CPU-win, GPU,
or paper-number gate without refreshed same-command packets.

Latest local follow-up: after merging the latest `origin/main`, rigid AVBD
point-joint and distance-spring extraction now carries the already-validated
projectable endpoint transform/mass/static metadata into the append step. The
appenders still fall back to registry lookup for hand-built test inputs, but
the production contact-stage source-row path no longer refetches the same
endpoint metadata immediately after extraction. Focused validation rebuilt
`dart_simulation_tests` and ran the full 83-test `test_avbd_rigid_block`
binary. While validating the merged branch, the clean no-work `World::step()`
and clean kinematics-graph shortcuts were also made profiling-aware, so normal
steps still take the fast path but built-in step profiling records the expected
pipeline and nested graph profiles. Focused validation also ran the full
`test_world_step_profiling` binary plus the clean-kinematics skip regression.
This is another pair-constraint/profiling-surface cleanup only; no source-row
CPU, GPU, or paper-number gate is closed without refreshed same-command
packets.
The AVBD rigid benchmark target also rebuilt, and a short diagnostic smoke over
the Motor, Rope, Hanging Rope, Spring, and Spring Ratio 2D source rows completed
under load average around 5.7-6.2; the smoke is intentionally non-packet
evidence.

Latest local follow-up: `AvbdScalarRowInventory` now has a generated-descriptor
sync path that compares cheap row keys for stable row order and only
materializes full descriptors once per active row on the same-order fast path.
The large rigid point-joint, motor, and distance-spring row builders use that
path, avoiding a per-step descriptor-vector allocation for large stable source
rows such as Joint Grid, rope, and soft-body fixed-joint lattices while keeping
the existing descriptor-vector fallback for row reorder/drop cases. Focused
validation rebuilt `dart_simulation_tests`, ran the new inventory regression,
the full `test_avbd_constraint` binary, the full 83-test
`test_avbd_rigid_block` binary, rebuilt `bm_avbd_rigid_fixed_joint`, and
smoke-ran the open spring/rope/Joint Grid/soft-body dashboard rows. The smoke
was host-clock/load sensitive and is not a same-command packet refresh, so no
source-row CPU, GPU, or paper-number gate is closed.

Latest local follow-up: large AVBD point-joint row builders now reserve their
heap scratch lazily only after finding a linear or angular axis that actually
emits a row. Linear-only spherical/rope-style joint sets still clear stale
angular inventories, but they no longer pre-reserve angular active-row and
descriptor vectors before discovering that every angular mask is empty. Focused
validation rebuilt `dart_simulation_tests`, ran the new large linear-only
rigid-block regression plus neighboring point-joint mask/spherical filters, ran
the full 83-test `test_avbd_rigid_block` binary, rebuilt
`bm_avbd_rigid_fixed_joint`, and smoke-ran the 2D/3D rope/net linear-only
source rows. The benchmark smoke stayed noisy and non-packet, so this is only a
row-builder allocation cleanup; no source-row CPU, GPU, or paper-number gate is
closed.

Latest local follow-up: the contact-stage AVBD pair-constraint hot path now can
extract point-joint and distance-spring anchors directly in body-local
coordinates for the private production `World::step()` path, avoiding redundant
world-anchor reconstruction immediately before the row appenders convert those
anchors back to local space. The public extraction helpers still default to
world-space anchors, while the local-only path is pinned by rigid-block
regressions including a nonzero current-pose fixed-joint anchor. Focused
validation rebuilt `dart_simulation_tests`, ran the focused AVBD rigid-block
filters plus the full 82-test `test_avbd_rigid_block` binary, and rebuilt/
smoke-ran the affected source-demo benchmark rows. Benchmark diagnostics were
mixed and CPU-frequency sensitive, so no Soft Body, Spring, Motor, GPU, or
paper-number gate is closed by this cleanup.

Latest local follow-up: clean no-work default `World::step()` paths now skip the
built-in step pipeline when the prepared schedule has no advanceable rigid-body,
multibody, deformable, joint, or loop-closure work and all frame caches are
clean. `WorldKinematicsGraph` also skips executing its compute graph when frame
caches are already clean. Focused coverage in
`World.StepSkipsCleanKinematicsGraph` proves counted clean steps still advance
time/frame without graph execution and that dirtying a parent frame re-enables
the graph and refreshes child transforms. The refreshed
`avbd-demo2d-ground-packet.json` records a 24-frame headless visual capture,
`BM_AvbdDemo2dGroundStep` at 16.3 ns median CPU time per step, and fresh native
`avbd-demo2d` Ground timing at 24.5 ns/step, so the narrow Ground CPU-win gate
is closed at about 1.51x faster than native. Broad ground-contact, GPU, and
paper-number gates are still open.

Latest local follow-up: the AVBD articulated high-ratio benchmark helper now
also exposes `BM_AvbdPaperScaleHighRatioChainStep`, a 50-link/50,000:1
variational-chain dashboard row using the same configured `World::step()`
solve-budget envelope as `PaperScaleHighRatioChainStaysFiniteAndResets`
(`200` iterations, `1e-9` tolerance). The performance dashboard dry-run and
display-name surfaces include the new row, and
`avbd_paper_scale_high_ratio_chain` plus
`avbd-paper-scale-high-ratio-chain-packet.json` now record an 8-frame headless
visual capture and benchmark packet for the same 50-link/50,000:1 smoke. This
does not add a same-hardware paper-number comparison, GPU path, or broad
source-corpus completion.

Latest local follow-up: the six public non-motor breakable fixed/spherical
py-demos now expose a testable weak re-arm hook matching their UI controls.
Focused integration coverage for the free-rigid fixed, free-rigid spherical,
world-link articulated fixed, same-multibody articulated fixed, world-link
articulated spherical, and same-multibody articulated spherical demos now
breaks the weak joint, resets at the high break force, verifies re-engagement,
then re-arms the weak break force and verifies the joint breaks again. Focused
validation ran those six pytest targets with the built dartpy extension path.
This is narrow user-facing breakable-joint lifecycle evidence only; broad
fracture corpus, GPU, and paper benchmark gates remain open.

Latest local follow-up: the four public articulated breakable velocity-motor
py-demos now carry mutable command state for their reset lifecycle. The
same-multibody revolute, same-multibody prismatic, world-anchored prismatic,
and world-anchored revolute break/reset integration tests now reset at a high
break force, reverse the target velocity, and verify the re-engaged motor rows
follow that updated command while the hard rows stay constrained, then re-arm
the weak break force and verify the same updated-command path breaks again.
Focused validation ran those four pytest targets with the built dartpy
extension path. This is narrow user-facing motor lifecycle evidence only; broad
motor, articulated fracture/source-corpus, GPU, and paper benchmark gates remain
open.

Latest local follow-up: the `avbd_demo2d_fracture` and
`avbd_demo3d_breakable` source-demo rows now have focused break/reset lifecycle
coverage in `test_demos_cycle.py`. The new regressions step each public
fixed-joint source row until most of the chain joints break, raise every joint's
reset break force, reset the broken rows, and verify the rows stay unbroken
while their anchor residuals contract again. Focused validation ran both new
pytest targets with the built dartpy extension path. This closes only the
narrow 2D Fracture/3D Breakable source-demo fixed-joint lifecycle evidence;
broad motor, articulated fracture/source-corpus, GPU, and paper benchmark gates
remain open.

Latest local follow-up: private generated current-pose movable-pair fixed
all-axis breakage now also has simulation-mode binary save/load/reset coverage.
The regression saves after generated fixed rows are marked broken, reloads,
verifies restored endpoints plus captured anchors, lets the broken anchor and
relative orientation drift, then resets breakage and checks the generated
linear and angular hard rows both re-engage. Focused validation rebuilt
`test_variational_integration` and ran the new fixed save/load filter plus the
full eight-test generated current-pose movable-pair lifecycle filter. This
completes narrow generated fixed/revolute/prismatic/spherical movable-pair
persistence coverage only; broad motor, fracture, source-corpus, GPU, and paper
benchmark gates remain open.

Latest local follow-up: private generated current-pose movable-pair spherical
linear-row breakage now also has simulation-mode binary save/load/reset
coverage. The regression saves after the generated spherical row is marked
broken, reloads, verifies restored endpoints and captured anchors, lets the
broken anchor drift while free relative orientation changes, then resets
breakage and checks only the linear rows re-engage. Focused validation rebuilt
`test_variational_integration` and ran the new spherical save/load filter plus
the neighboring spherical current-pose reset filter. This is narrow private
generated spherical lifecycle coverage only; broad motor, fracture,
source-corpus, GPU, and paper benchmark gates remain open.

Latest local follow-up: private generated current-pose movable-pair revolute
and prismatic velocity rows now have simulation-mode binary save/load/reset
regressions. The tests create private non-topology `Joint` entities, let
simulation entry capture anchors and non-cardinal free-axis bases, save after
breakage, reload, verify restored endpoint/type/axis/actuator command and
effort-limit state plus captured anchors, let broken rows drift, then reset
breakage with an updated command and check hard rows plus free-axis motor motion
re-engage across two movable links. Focused validation rebuilt
`test_variational_integration` and ran the new two-test filter plus the
neighboring four-test current-pose movable-pair lifecycle filter. `origin/main`
was fetched through a one-off HTTPS fetch after SSH was unavailable, and
`git merge origin/main` was a no-op because the branch was already current.
This is narrow private generated one-DOF lifecycle coverage only; broad motor,
fracture, source-corpus, GPU, and paper benchmark gates remain open.

Latest local follow-up: public same-multibody movable-pair revolute and
prismatic articulated velocity motors now have simulation-mode binary
save/load/reset regressions. The tests reload a broken joint, verify restored
endpoint/type/axis/actuator command and effort-limit state, let broken rows
drift under opposing endpoint forces, then reset breakage with an updated
command and check the hard rows plus free-axis motor motion re-engage across
two movable links. Focused
validation rebuilt `test_variational_integration`, ran the movable-pair
fixed/revolute/prismatic save/load and break/reset filters, and ran
`pixi run lint`. This is narrow public one-DOF lifecycle coverage only; broad
motor, fracture, source-corpus, GPU, and paper benchmark gates remain open.

Previous local follow-up: rigid AVBD point-joint and distance-spring extraction
and append paths now use the projectable-body metadata helper, so the hot path
no longer classifies each endpoint and then fetches the same transform a second
time, and no longer materializes no-op static-static pair constraints. Rigid
contact-snapshot body insertion also reuses the already-checked projectable
transform, mass, and static tag when materializing new body state. Focused
validation rebuilt `dart_simulation_tests`, ran the rigid block
static-static/pair-constraint extraction, point-joint, and distance-spring
filters, and ran `pixi run lint`. This remains an overhead cleanup only; the
source-demo CPU-win packet gates stay open until refreshed same-command
source-runner comparisons beat the native rows.

Previous local follow-up: articulated point-joint extraction now builds a
per-multibody link-index cache before scanning AVBD point-joint configs, so
same-multibody and world-link private rows no longer rediscover endpoint
ownership by scanning every structure/link for every joint. Focused validation
rebuilt `dart_simulation_tests`, ran
`test_variational_integration --gtest_filter='VariationalIntegration.AvbdPublicArticulated*:VariationalIntegration.PaperScaleHighRatioChainStaysFiniteAndResets'`,
ran `test_world --gtest_filter='World.ArticulatedPointJoints*'`, rebuilt the
`bm_avbd_rigid_fixed_joint` benchmark target after the staged `WorldOptions`
layout change, and smoke-ran articulated `/32` motor/joint rows plus
`BM_AvbdArticulatedHighRatioChainStep`. The source-demo CPU-win packet gates
remain open because this is an articulated extractor hot-path cleanup, not a
refreshed same-command source-runner comparison.

Previous local follow-up: private articulated two-movable-link lifecycle coverage
now includes direct revolute motor reset, current-pose fixed all-axis reset,
current-pose revolute/prismatic motor reset, spherical current-pose
linear-row reset, non-cardinal current-pose revolute/prismatic motor-axis
coverage, current-pose revolute/prismatic finite-limit coverage, public
same-multibody/world-anchored articulated revolute/prismatic
floating-endpoint plus selected off-origin-anchor facade non-cardinal axis
coverage, selected same-multibody/world-link facade save/load non-cardinal
axis-basis persistence, selected one-DOF broken-state save/load/reset
non-cardinal axis-basis persistence, selected direct one-DOF break/skip/reset
non-cardinal axis-basis coverage, public same-multibody/world-link one-DOF
non-cardinal off-origin finite-limit coverage, and public same-multibody movable-pair
non-cardinal motor-axis and finite-limit coverage. The dartpy fixed point-joint
break/skip/reset regression now also asserts the same-multibody and world-link
fixed endpoint shapes while exercising explicit all-axis anchors, and the
dartpy spherical break/skip/reset regressions now assert same-multibody and
world-link endpoint shapes while keeping orientation free. The dartpy
one-DOF motor break/skip/reset regressions now also assert public endpoint and
axis shapes for same-multibody, world-link, and movable-pair revolute/prismatic
cases. The C++ same-multibody fixed/revolute/prismatic and world-link
spherical/revolute/prismatic save/load regressions now also assert restored
endpoint shape before rebuilding the private AVBD rows.
The same-multibody/world-link revolute/prismatic broken-state save/load
regressions now also assert restored one-DOF motor effort limits alongside the
velocity actuator command state.
`PaperScaleHighRatioChainStaysFiniteAndResets` now exercises a
50-link/50,000:1 articulated chain through configured `World::step()`
variational solve-budget fields (`200` iterations, `1e-9` tolerance) and
verifies finite rollout plus reset. This is stability evidence only; benchmark
packet parity remains open.
The new fixed current-pose regression builds a same-multibody private
`comps::Joint`, lets simulation entry generate the hard AVBD config from the
current pose, verifies break-force marking skips later rows while opposing
endpoint forces separate/rotate the captured pair, then clears the broken state
and checks the captured anchor and relative orientation re-engage. The companion
revolute current-pose regressions reverse the motor command to check
anchor/hinge rows plus free-axis motor re-engagement, and the generated
current-pose drive regressions now use non-cardinal revolute/prismatic axes and
check the generated free-axis basis column. The public same-multibody and
world-anchored articulated revolute/prismatic floating-endpoint and
selected off-origin-anchor drive regressions now use non-cardinal axes and
assert the facade-generated AVBD free-axis basis column before stepping. Public
save/load regressions now also preserve non-cardinal same-multibody/world-link
revolute axes and the world-link prismatic axis, then assert the rebuilt AVBD
free-axis basis columns after simulation entry. Public one-DOF broken-state
save/load/reset regressions now also preserve non-cardinal
same-multibody/world-link revolute/prismatic axes through the binary
round-trip, assert the reset-rebuilt AVBD basis columns, and verify free-axis
motion resumes after reset. Public direct same-multibody/world-link
revolute/prismatic one-DOF break/skip/reset regressions now also use
non-cardinal axes, assert the generated AVBD basis columns, and verify the
free-axis motor rows keep progressing after reset while the masked hard rows
re-engage. Public same-multibody/world-link revolute/prismatic tiny-limit
facade regressions now also use non-cardinal axes, assert the generated AVBD
basis columns, and prove tiny effort caps keep the free-axis motion far below
the commanded unbounded target. Public
same-multibody movable-pair revolute/prismatic facade
regressions now also use non-cardinal axes, assert the generated AVBD basis
columns, and cap the velocity motors with tiny effort limits to prove both
movable endpoints preserve the hard anchor/hinge/masked rows without turning
the free axis into an unbounded target. The companion finite-limit regressions
cap generated velocity motors with tiny effort limits to prove
generated free-axis rows honor bounds without turning into hard unbounded
velocity targets. The synthetic current-pose extractor fixtures now also mark
`hasAvbdStiffnessState = false` so they exercise endpoint default inference
rather than the serialized stiffness-state defaults. Focused validation ran
`pixi run build-simulation-tests` plus the current-pose extractor, public
same-multibody/world-anchored floating-endpoint/off-origin-anchor facade
non-cardinal axis, selected save/load and broken-state save/load/reset
non-cardinal axis-basis, selected direct break/skip/reset non-cardinal
axis-basis, public same-multibody/world-link one-DOF non-cardinal
finite-limit, public
same-multibody movable-pair non-cardinal motor-axis and finite-limit, and
movable-pair fixed/revolute/prismatic/spherical lifecycle plus explicit-anchor
dartpy fixed/spherical endpoint-shape, one-DOF endpoint/axis-shape, restored
same-multibody/world-link save/load endpoint-shape, restored one-DOF
effort-limit state, and revolute/prismatic tiny-limit filters.
This is narrow articulated lifecycle and axis-basis evidence only; it does not
close broad motor, fracture, source-corpus, GPU, or paper benchmark gates.

Previous local follow-up: rigid AVBD row-index layouts now stay warm across
frames, single-family point-pair/angular solves route directly without copying
into combined work vectors, one-new-row point-joint/distance-spring appends skip
endpoint row-counter hash-map setup, and small point-joint/motor/distance-spring
row builders use stack descriptor/active-row storage for up to 16 candidate rows
instead of allocating temporary vectors. The contact stage also extracts and
appends point-joint and distance-spring families independently, so Motor rows
do not scan spring configs and Spring rows do not scan point-joint configs, and
the snapshot solve now clears absent row-family inventories directly instead
of calling empty contact/joint/motor/spring builders. Focused validation ran
`pixi run build-simulation-tests` and the rigid row-driver/contact-stage
`test_avbd_rigid_block` filters. Local profile/benchmark smoke reports
`BM_AvbdDemo2dMotorStep_median` around 8.43 us,
`BM_AvbdDemo2dSpringStep_median` around 3.98 us, and
`BM_AvbdDemo2dSpringRatioStep_median` around 36.16 us, but this remains
overhead cleanup rather than refreshed packet evidence or a CPU-win claim.

Previous local follow-up: the rigid AVBD contact stage now skips point-joint and
distance-spring input extraction when the registry has no AVBD pair-constraint
configs, and the contact-enabled AVBD path only extracts/appends those rows
when such configs exist. The split rigid-body velocity stage also now assembles
force batches only for advanceable rigid bodies, leaving the all-body force
batch path in the standalone batched SoA integration stage intact. Focused
coverage in
`AvbdRigidBlock.RigidWorldPairConstraintConfigPresenceGuardsExtraction` checks
the new registry guards and that stale extraction vectors are cleared, and
`World.RigidBodyVelocityStageSkipsPrescribedBodyForces` keeps static/kinematic
force immunity plus dynamic force/gravity behavior covered. The `.gitignore`
comment now also makes explicit that `avbd-*-packet.json` files are curated
plan/test fixtures while other JSON dumps in the PLAN-104 folder stay ignored.
This is overhead cleanup only; it is not packet evidence or a CPU-win claim.

Previous local follow-up: the rigid AVBD contact stage now reuses a scratch
contact snapshot through `buildAvbdRigidWorldContactSnapshotInto`, avoiding
per-step contact-snapshot vector/map reallocation while preserving the
value-returning helper for tests and standalone callers. Focused coverage in
`AvbdRigidBlock.RigidWorldContactSnapshotBuildIntoClearsReusedState` proves the
in-place builder clears stale joint/motor/spring state and preserves reused
vector capacity. Same-command benchmark smoke over selected 2D contact-heavy
source rows was mixed and host-load sensitive, so this is not packet evidence
or a CPU-win claim.

Previous local follow-up: focused serialization coverage now reloads broken
public free-rigid fixed, revolute, prismatic, and spherical joints in
simulation mode, verifies each restored joint stays broken and no longer
filters overlapping endpoint contacts, then checks `resetBreakage()` restores
contact filtering. Same-multibody and world-link articulated fixed, revolute,
prismatic, and spherical point joints now also have a focused simulation-mode
binary round-trip regression for broken-state reset; the revolute/prismatic
cases preserve velocity-actuator command and effort-limit state across the
round trip. Public AVBD point-joint start/linear/angular stiffness facade state
now lives on the serialized `Joint` component, with free-rigid and articulated
design-mode binary save/load coverage proving those setters still work before
simulation entry rebuilds the private AVBD rows. Direct public facade validation
now also covers same-multibody and world-link articulated fixed, revolute,
prismatic, and spherical start/linear/angular stiffness defaults, finite
setters, and invalid setter rejection from both C++ and dartpy, plus C++/dartpy
same-link, cross-multibody, and cross-world endpoint-ownership rejection for the
public articulated point-joint facade. A focused legacy v15 binary metadata
regression also keeps solver-family options loading before the v16
ignored-collision-pair tail. The curated AVBD plan packet JSON files are
intentionally trackable as `avbd-*-packet.json`, while scratch JSON in that plan
folder is ignored by `.gitignore`.

Previous local follow-up: private free-rigid AVBD linear motor rows now sit beside
the existing angular motor rows in the 6-DOF rigid kernel. Public free-rigid
prismatic velocity actuators now extract to those bounded linear motor rows,
the contact-stage motor inventory handles linear and angular rows together, and
the branch now adds the matching `avbd_rigid_prismatic_motor` py-demo,
`BM_AvbdRigidPrismaticMotorStep` dashboard row, packet writer, and tracked
`avbd-rigid-prismatic-motor-packet.json` evidence packet. Focused C++ coverage
checks the linear builder, row solve direction, and public World extraction
while dartpy coverage steps the public prismatic velocity motor from Python.

Previous local follow-up: focused C++ coverage now reloads a broken public fixed
point joint between two movable same-multibody links, verifies the restored
broken joint stays skipped, and resets it to rebuild the explicit AVBD anchor
rows. Focused dartpy coverage now also steps that movable same-multibody fixed
break/reset path plus movable same-multibody revolute/prismatic motor
break/reset paths through the public Python facade, and focused C++ coverage now
exercises the same explicit off-origin movable-pair revolute/prismatic motor
break/reset paths. The row-inventory/contact
follow-up also has AVBD scalar row inventories
now warm-start stable
same-order descriptor lists in place instead of rebuilding the previous-row
`std::map` every step, preserving the existing map path for dropped or reordered
rows. Rigid World contact snapshots now also assign contact/joint/spring row
ordinals through reserved endpoint-pair hash counters instead of per-step tree
maps. The rigid row driver now also leaves per-body row-index scratch tables
unbuilt for row families that are absent in the current solve, avoiding
attachment/friction/spring table setup on joint-only rows. Append paths now
also seed the snapshot body-index cache when a fallback scan finds a preseeded
entity. Focused coverage checks same-order warm-start semantics, resets per-row
scratch direction, preserves repeated endpoint-pair row ordinals across
multiple append calls, caches fallback body indices without duplicating bodies,
and keeps unused row-index scratch empty while point-pair and distance-spring
rows still solve. Narrow benchmark smoke remains thermal/frequency sensitive,
so it is not packet evidence or a CPU-win claim. Keep the checked source-row
packet CPU gates open until refreshed same-command packets beat the native
runners.

The previous local follow-up ported the `avbd-demo3d` Soft Body source row
through the public rigid-body fixed-joint facade using finite AVBD linear and
angular material stiffness. The new `avbd_demo3d_soft_body` py-demo matches the
source `sceneSoftBody` row at revision `7701bd427d55` with one static ground,
three 4x4x4 dynamic rigid-box lattices, 432 finite-stiffness all-axis fixed
joints (`Klin=1000`, `Kang=250`), and 648 diagonal ignored collision pairs.
`BM_AvbdDemo3dSoftBodyStep`,
`scripts/run_avbd_demo3d_reference_timing.py --scene soft_body`,
`scripts/write_avbd_demo3d_soft_body_packet.py`, focused Python source-row and
packet tests, and
`docs/plans/104-vertex-block-descent-solver/avbd-demo3d-soft-body-packet.json`
provide the visual/DART-benchmark/native-reference evidence. The packet records
DART at 3.93 ms/step median versus 4.76 ms/step for the native source runner
on this host, so the narrow CPU reference-comparison gate is closed while GPU
parity, broader soft/rigid coupling, and paper-number gates remain open.

The AVBD paper, project page, 2D demo source, and 3D demo source were inspected
and folded into PLAN-104. The implementation now has an internal scalar row
utility, deterministic row-key/inventory warm-start cache, active half-space
contact-normal block stamping, a standalone mass-spring AVBD contact-normal
driver, scalar hard point-attachment row stamping, a standalone mass-spring AVBD
attachment driver, scalar finite-stiffness spring rows, and narrow opt-in World
VBD paths for active static half-space contact-normal rows, pinned/scripted hard
attachment rows, and progressive finite-stiffness spring rows in supported
serial mass-spring scenes. Supported contact-normal, friction-tangent,
attachment, and finite-stiffness spring requests now run together through one
combined serial mass-spring AVBD row solve. The implementation also wires
finite-stiffness tetrahedral material rows into supported serial, frictionless
pure-tet World scenes, using a dimensionless material scale, strain-norm row
error, separate tet-row diagnostics, and coexistence with the existing lagged
VBD self-contact penalty. Contact-free finite-stiffness mass-spring scenes with
an inert material friction coefficient now stay on the AVBD row path instead of
falling back when there is no active contact or self-contact friction source. A
bounded friction-tangent row primitive now
participates in the serial mass-spring AVBD row driver, and supported
static-contact mass-spring World scenes generate two tangent rows per active
contact-normal row. Static box obstacle contact rows now include face/edge/corner
feature IDs so normal and friction rows reset when the contacted box manifold
changes while still warm-starting same-feature penetrations, and persisting
static half-space friction rows project their decayed dual into the current
tangent basis when smooth obstacle normals change. Persisting self-contact
friction rows project their generalized tangential dual into the current 12D
tangent stencil. Adjacent tangent-row pairs now use the lagged tangential dual to
switch between static sticking and dynamic sliding and project the paired force
to the circular Coulomb cone.
Supported serial mass-spring
self-contact scenes can also generate one AVBD self-contact normal row per
lagged point-triangle / edge-edge primitive, stamp each incident local vertex
through the shared scalar row, and report `vbdAvbdSelfContactNormalRows`.
Self-contact friction tangent rows now reuse lagged point-triangle / edge-edge
tangent stencils in the combined mass-spring row driver, with supported World
generation for serial mass-spring self-contact scenes and focused coverage for
tangent direction, static/dynamic pair switching, circular-cone projection,
tangential-motion resistance, World row generation, and World-level tangential
slip reduction. Static-contact and self-contact friction rows also have a
combined-row coexistence regression so both row families can participate in the
same supported AVBD mass-spring solve. Pure tetrahedral finite-stiffness scenes
can now also combine AVBD self-contact normal rows and matching self-contact
friction tangent rows in the same serial tet solve when
`useAvbdSelfContactNormalRows` is requested, with row-counter coverage for tet
finite-stiffness, self-contact normal, and self-contact friction families.
Unsupported mixed spring-plus-tet, mass-spring self-contact without the
self-contact AVBD flag, Chebyshev, Rayleigh-damped, parallel, and
unsupported-row requests have explicit fallback coverage that keeps them on the
existing VBD path. The first internal rigid contact-stage AVBD activation is
now guarded by `RigidAvbdContactConfig`: supported free rigid-body contacts use
the private World-contact snapshot and 6-DOF row solve as a velocity projection
consumed by the standard rigid position stage, while unsupported envelopes fall
back to sequential impulses. The private rigid contact snapshot also derives box
face/edge/corner endpoint feature IDs plus cylinder side/cap/rim and capsule
side/top-cap/bottom-cap endpoint feature IDs, scopes row ordinals per canonical
endpoint pair, and now assigns same-feature manifold rows from deterministic
canonical-local contact-point ordering so narrow-phase contact order changes do
not swap warm-start state. Focused private and live `World::collide()`
endpoint-order coverage now also proves swapped contact body endpoints keep the
same row ordinals and warm-started normal state for both sphere/plane contact
points and live box-box manifolds. Actual `World::collide()` sphere/plane
contact-point replay and live box-box manifold box-face feature coverage remain
in place.
Known and unknown-index fallback contacts now map world points through body and
collision-shape local transforms before rigid feature coding, while explicit
endpoint-A and endpoint-B compound shape-index contacts use the narrow-phase
shape-local contact point before feature coding, with actual `World::collide()`
sphere/cylinder/capsule/plane/mesh contacts now covered through primitive
feature IDs, including shape-scoped sphere body features.
Explicit and inferred offset shape contacts persist the shape-frame face/edge
identity; unique compound
plane contacts now derive
shape-scoped face IDs, triangle-mesh contacts now derive face/edge/vertex
feature IDs, and ambiguous compound contacts keep the body-level fallback.
Private rigid contact friction rows now also store previous tangent directions
and project the warm-started paired dual into the current tangent basis when
the contact normal rotates. The private rigid block path also has point-joint linear,
angular, and combined row builders for fixed-anchor
translation and orientation constraints, with step-start previous constraint
values seeded for AVBD alpha regularization. Those private point-joint rows can
now be appended to the World rigid snapshot/solve/apply wrapper and combined
step helper from world-space point-joint inputs; a private fixed-joint ECS
extractor plus step-helper overload now covers rigid-body-linked joint
entities, and the internal contact-stage AVBD opt-in can now project those
fixed-joint rows with or without active contacts. A private helper now derives
the AVBD fixed-joint config from the current rigid-body pair pose, preserving
the child-origin anchor and relative orientation for rigid-body-linked fixed
joints while rejecting multibody link endpoints. Public multibody joint
extraction is still not wired. The current local slice adds private per-axis
linear and angular masks to those point-joint builders, preserving all-axis
fixed-joint behavior while letting later hinge/revolute and limited-DOF configs
reuse the same descriptors, row inventory, and alpha-regularized warm starts.
It also adds named private revolute and prismatic point-joint builders that
construct arbitrary joint-axis bases, leave the hinge or translation axis free,
and preserve configured axes/masks through World point-joint input, snapshot
assembly, and solve coverage. Simulation-entry current-pose initialization and
extraction now also cover private rigid-body ECS revolute/prismatic joint
entities, deriving the same axes/masks from their configured joint axis while
the private articulated bridge now accepts explicitly hard masked multibody-link
point-joint configs for AVBD revolute/prismatic rows through the variational
loop-constraint projection path. The private endpoint layer now classifies free
rigid bodies separately from multibody links, and hard fixed multibody-link
point-joint configs can bridge into the variational articulated solve path.
Explicitly hard breakable multibody-link point-joint configs also enter that
projection path and mark their source joint broken from the projection load, so
future steps skip them.
Public DART 7 `World` facades now expose free rigid-body
revolute and prismatic joints through C++ and dartpy, with generated stubs,
focused C++/Python tests, and the categorized `sx_rigid_limited_joints`
py-demo. This branch also adds the first AVBD-specific `py-demos`
rigid-constraint scene, `avbd_rigid_fixed_joint_contact`, so users can inspect
the public fixed-joint row path while ordinary rigid contact acts on the
payload. The current follow-up adds private angular/linear motor rows and rigid
fracture dual-threshold/reset helpers in the 6-DOF row kernel, then wires
public rigid-body revolute and prismatic velocity actuators into those bounded
AVBD motor rows with persistent contact-stage motor inventory, focused C++
coverage, and dartpy public prismatic stepping coverage. The categorized
`avbd_rigid_revolute_motor` and `avbd_rigid_prismatic_motor` py-demos plus
tracked visual/benchmark packets cover those narrow free-rigid motor paths.
The AVBD dashboard benchmark target also has end-to-end public-World
`BM_AvbdRigidRevoluteMotorStep` and `BM_AvbdRigidPrismaticMotorStep` rows for
those motor paths. The dashboard's
bounded AVBD World slice now also includes `BM_AvbdEmptyWorldStep` for the empty
source-corpus baseline with source-default timestep/gravity metadata and a
tracked `avbd-empty-baseline-packet.json` visual/benchmark packet,
`BM_AvbdDemo2dGroundStep` for the static `avbd-demo2d` Ground source-row port,
`BM_AvbdDemo2dMotorStep` for the first one-DOF motor `avbd-demo2d` Motor
source-row port,
`BM_AvbdDemo2dHangingRopeStep` for the `avbd-demo2d` Hanging Rope source-row
port,
`BM_AvbdDemo2dFractureStep` for the `avbd-demo2d` Fracture source-row port,
`BM_AvbdDemo2dDynamicFrictionStep` for the `avbd-demo2d` Dynamic Friction
source-row port,
`BM_AvbdDemo2dStaticFrictionStep` for the `avbd-demo2d` Static Friction
source-row port,
`BM_AvbdDemo2dPyramidStep` for the `avbd-demo2d` Pyramid source-row port,
`BM_AvbdDemo2dCardsStep` for the `avbd-demo2d` Cards source-row port,
`BM_AvbdDemo2dStackStep` for the `avbd-demo2d` Stack source-row port,
`BM_AvbdDemo2dStackRatioStep` for the `avbd-demo2d` Stack Ratio source-row port,
`BM_AvbdDemo2dRodStep` for the `avbd-demo2d` Rod source-row port,
`BM_AvbdDemo2dJointGridStep` for the `avbd-demo2d` Joint Grid source-row port,
`BM_AvbdDemo2dRopeStep` for the `avbd-demo2d` Rope source-row port,
`BM_AvbdDemo2dHeavyRopeStep` for the `avbd-demo2d` Heavy Rope source-row port,
`BM_AvbdDemo2dNetStep` for the `avbd-demo2d` Net source-row port,
`BM_AvbdDemo3dGroundStep` for the `avbd-demo3d` Ground source-row port,
`BM_AvbdDemo3dDynamicFrictionStep` for the `avbd-demo3d` Dynamic Friction
source-row port,
`BM_AvbdDemo3dStaticFrictionStep` for the `avbd-demo3d` Static Friction
source-row port,
`BM_AvbdDemo3dPyramidStep` for the `avbd-demo3d` Pyramid source-row port,
`BM_AvbdDemo3dRopeStep` for the `avbd-demo3d` Rope source-row port,
`BM_AvbdDemo3dHeavyRopeStep` for the `avbd-demo3d` Heavy Rope source-row port,
`BM_AvbdDemo3dStackStep` for the `avbd-demo3d` Stack source-row port,
`BM_AvbdDemo3dStackRatioStep` for the `avbd-demo3d` Stack Ratio source-row port,
`BM_AvbdDemo3dBridgeStep` for the `avbd-demo3d` Bridge source-row port,
`BM_AvbdDemo3dBreakableStep` for the `avbd-demo3d` Breakable source-row port,
`BM_AvbdArticulatedRevoluteMotorStep` and
`BM_AvbdArticulatedPrismaticMotorStep` rows for the public articulated
one-DOF velocity motor paths, with the revolute `/1` row backed by
`avbd-articulated-revolute-motor-packet.json` and the prismatic `/1` row
backed by `avbd-articulated-prismatic-motor-packet.json`,
`BM_AvbdArticulatedBreakableMotorStep` for the active break-force armed
articulated motor path with
`avbd-articulated-breakable-motor-packet.json` packet evidence,
`BM_AvbdArticulatedPrismaticBreakableMotorStep` for the active break-force
armed articulated prismatic motor path with
`avbd-articulated-prismatic-pair-breakable-motor-packet.json` packet evidence,
`BM_AvbdArticulatedWorldPrismaticBreakableMotorStep` for the active
world-anchored articulated prismatic motor path with
`avbd-articulated-world-prismatic-breakable-motor-packet.json` packet evidence,
`BM_AvbdArticulatedWorldRevoluteBreakableMotorStep` for the active
world-anchored articulated revolute motor path with
`avbd-articulated-world-revolute-breakable-motor-packet.json` packet evidence,
plus
`BM_AvbdRigidBreakableJointStep` and
`BM_AvbdArticulatedBreakableJointStep` rows for the public free-rigid and
articulated breakable fixed point-joint paths with
`avbd-rigid-breakable-joint-packet.json` and
`avbd-articulated-breakable-joint-packet.json` packet evidence, plus
`BM_AvbdRigidSphericalBreakableJointStep`,
`BM_AvbdArticulatedWorldSphericalBreakableJointStep`, and
`BM_AvbdArticulatedSphericalPairBreakableJointStep` rows with matching
spherical packet evidence. The narrow public fracture
lifecycle now exposes
`Joint` break-force and broken-state accessors,
marks free rigid-body AVBD point joints broken when solved row load exceeds the
threshold, skips broken joints on later extraction, and adds a categorized
`avbd_rigid_breakable_joint` py-demo with break, release, reset, and captured
fixed-pose re-engagement coverage plus
`avbd_rigid_spherical_breakable_joint` py-demo coverage for free-rigid spherical
break/reset with orientation left free. The current follow-up adds
`avbd_articulated_revolute_motor` and `avbd_articulated_prismatic_motor`,
public articulated one-DOF velocity-motor py-demos with command reversals
through the variational articulated bridge, plus
`avbd_articulated_motor_breakable_joint`, a same-multibody public articulated
revolute motor break/reset py-demo over that bridge, plus
`avbd_articulated_prismatic_motor_breakable_joint`, a world-anchored public
articulated prismatic motor break/reset py-demo over that bridge, plus
`avbd_articulated_breakable_joint`, a public articulated fixed point-joint
break/reset py-demo over that bridge, plus
`avbd_articulated_spherical_breakable_joint`, a public articulated spherical
point-joint break/reset py-demo over the linear-only row bridge, now backed by
`avbd-articulated-spherical-breakable-joint-packet.json` packet evidence. It
also adds
focused C++ regressions for
public same-multibody articulated revolute/prismatic velocity motors, explicit
local/world anchor bindings with same-multibody and world-anchored
fixed/revolute/prismatic off-origin regressions, and fixed break/reset
lifecycle when both point-joint endpoints are movable floating links. Public
same-multibody articulated revolute/prismatic velocity motors also now mark
their source joint broken and skip later hard/motor rows in focused C++
regressions. Public articulated spherical point-joint facades now expose
linear-only pinned-anchor rows through C++/dartpy/stubs, preserving the
captured anchor while leaving orientation free in focused C++ and Python
endpoint coverage, including explicit link-link and world-link anchor overloads.
Focused dartpy stepping coverage now also exercises the same-multibody and
world-link spherical break/skip/reset paths that re-pin the anchor without
adding orientation rows. Focused C++ coverage now
also saves and reloads same-multibody link-link and world-link spherical
facades, then verifies `enterSimulationMode()` rebuilds the private AVBD
linear-only rows from the serialized public joint state.
The current source-corpus follow-up adds `avbd_demo2d_rod`, a
matched-metadata `avbd-demo2d` Rod py-demo with 20 source-shaped rigid links,
19 all-axis fixed joints, focused Python source-row coverage,
`BM_AvbdDemo2dRodStep`, and
`avbd-demo2d-rod-packet.json`; that packet records DART about 9.58x slower than
the native source runner on this host, so the CPU-win gate remains open. The
current source-corpus follow-up also adds `avbd_demo2d_joint_grid`, a
matched-metadata `avbd-demo2d` Joint Grid py-demo with 625 source-shaped rigid
boxes, two static top-corner anchors, 1200 public all-axis fixed joints,
focused Python source-row coverage, `BM_AvbdDemo2dJointGridStep`, and
`avbd-demo2d-joint-grid-packet.json`; that packet records DART about 2.06x
slower than the native source runner on this host after reusing per-body
row-index scratch and caching snapshot body indices, so the CPU-win gate remains
open. The source's 1152 diagonal `IgnoreCollision` pairs are now configured
through DART's public per-pair collision filter. The latest source-corpus
follow-up adds `avbd_demo2d_soft_body`, a matched-metadata `avbd-demo2d` Soft
Body py-demo with one static ground slab, two 15x5 dynamic rigid-box lattices,
260 finite-stiffness all-axis fixed joints, 224 diagonal ignored collision
pairs, focused Python source-row coverage, `BM_AvbdDemo2dSoftBodyStep`, and
`avbd-demo2d-soft-body-packet.json`; that packet records DART about 6.30x
slower than the native source runner on this host, so the CPU-win gate remains
open. The current
source-corpus follow-up also adds `avbd_demo2d_cards`, a
matched-metadata `avbd-demo2d` Cards py-demo with one source-shaped ground slab,
40 thin source-shaped dynamic cards across five levels, focused Python
source-row coverage, `BM_AvbdDemo2dCardsStep`, and
`avbd-demo2d-cards-packet.json`; that packet records DART about 5.38x slower
than the native source runner on this host, so the CPU-win gate remains open.
This is still narrow lifecycle evidence only: no broad
articulated motor/fracture corpus, GPU path, paper-corpus demo, broad
motor/fracture/facade lifecycle coverage, or paper/reference benchmark packet is
complete.

## Current Branch

Current reality (2026-06-11): this checkout is
`avbd/source-row-extraction-precheck`, a stacked local branch that has just
merged `avbd/source-row-perf-slice` through #2977 head `5297462d34b` and
`origin/main` at `7d05d7b9ea7`. The latest handoff state, validation, local
branch inventory, and stash inventory are owned by
[`Fresh-Session Handoff (2026-06-11)`](#fresh-session-handoff-2026-06-11).
The parent #2977 branch is scoped to source-row contact-prepare overhead; this
stacked branch is for the next source-row extraction-precheck slice. The older
checkpoint and split-stack branch descriptions below are historical context and
should not be treated as the current checkout without re-verifying
`git status --short --branch` and the open PR heads.

`feature/avbd-articulated-masked-rows` - staged local slice based on cached
`origin/main` at `dbac6c63e9f`, including the scalar-row foundation,
mass-spring AVBD row
families, standalone tet-material rows, and World wiring for supported pure-tet
finite-stiffness material rows, plus supported World static-contact friction
tangent rows, static box feature IDs, static half-space tangent dual projection,
self-contact tangent dual projection, private rigid contact tangent dual
projection, supported World self-contact
normal/friction rows, combined static/self-contact friction row coexistence
coverage, and the first private 6-DOF rigid block plus contact/friction
point-pair row foundation. Private rigid point-pair friction tangent pairs now
reuse the deformable friction semantics for lagged-dual static/dynamic
switching and circular Coulomb-cone projection, and a private serial rigid row
driver now sweeps point attachments, contact-normal point pairs, and paired
friction tangent rows. A private rigid contact-manifold row builder now turns
active contact points with stable endpoint feature IDs into warm-started normal
and paired tangent rows for that driver. A private World-contact snapshot helper
now translates rigid-body `World::collide()` contacts into those
manifold-point inputs and verifies that they can drive the private serial rigid
row solve plus dynamic rigid-body ECS writeback through a combined private
wrapper. Persisting private rigid contact friction rows now project their
warm-started paired tangent dual into the current tangent basis when contact
normals rotate, preserving the physical tangent impulse across basis changes.
The current branch additionally adds the first internal
`RigidAvbdContactConfig` contact-stage activation for supported free rigid-body
contacts as a velocity-level projection and box-feature/pair-scoped rigid
contact row identity plus live sphere/plane and box-box manifold endpoint-order
row identity for the private snapshot path, plus private point-joint
linear, angular, and combined row builders with World snapshot/step
append/solve/apply coverage for world-space point-joint inputs and a private
fixed-joint ECS extractor plus step-helper overload for rigid-body-linked joint
entities, plus internal contact-stage velocity-projection coverage for those
fixed-joint rows with or without active contacts. This checkpoint adds a private
fixed-joint pose bridge that converts a rigid-body-linked fixed joint's current
pose into AVBD local anchors and target relative orientation, initializes
missing private AVBD fixed-joint configs for opt-in rigid bodies at simulation
entry, and adds regression coverage that multibody links are not silently
treated as rigid AVBD bodies. It also adds masked private point-joint row
generation for constrained linear/angular axes, plus named private
revolute/prismatic point-joint configs with arbitrary joint-axis bases and
private rigid-body ECS current-pose extraction for those one-DOF joint
entities. The current checkpoint adds public free rigid-body revolute/prismatic
facades, dartpy bindings/stubs, focused C++/Python coverage, and the
`sx_rigid_limited_joints` py-demo on top of that private path. This branch also
adds the `avbd_rigid_fixed_joint_contact` Python demo scene and catalog/test/doc
coverage for the first narrow user-visible AVBD rigid-constraint showcase. The
current local follow-up adds private angular-motor row construction, bounded
motor descriptors, solver-direction coverage, rigid fracture
dual-threshold/reset helpers, public rigid-body revolute/prismatic velocity
actuator extraction into AVBD angular/linear motor rows, persistent motor
inventory in the contact stage, focused free-rigid prismatic public stepping
coverage, the `avbd_rigid_revolute_motor` and `avbd_rigid_prismatic_motor`
py-demos, the `avbd-rigid-revolute-motor-packet.json` and
`avbd-rigid-prismatic-motor-packet.json` packets, and the AVBD dashboard
`BM_AvbdRigidRevoluteMotorStep` and `BM_AvbdRigidPrismaticMotorStep` benchmark
rows. The current benchmark follow-up
also adds `BM_AvbdArticulatedRevoluteMotorStep` and
`BM_AvbdArticulatedPrismaticMotorStep` rows to the same bounded AVBD World
dashboard slice for the public articulated motor paths and a tracked
`avbd-articulated-revolute-motor-packet.json` for the revolute `/1` row plus a
tracked `avbd-articulated-prismatic-motor-packet.json` for the prismatic `/1`
row, plus
`BM_AvbdArticulatedBreakableMotorStep` for the active break-force armed
articulated motor path with a tracked
`avbd-articulated-breakable-motor-packet.json` for the `/1` row,
`BM_AvbdArticulatedPrismaticBreakableMotorStep` for the active break-force
armed articulated prismatic motor path with a tracked
`avbd-articulated-prismatic-pair-breakable-motor-packet.json` for the `/1`
row, plus
`BM_AvbdArticulatedWorldPrismaticBreakableMotorStep` for the active
world-anchored articulated prismatic motor path with a tracked
`avbd-articulated-world-prismatic-breakable-motor-packet.json` for the `/1`
row, plus
`BM_AvbdArticulatedWorldRevoluteBreakableMotorStep` for the active
world-anchored articulated revolute motor path with a tracked
`avbd-articulated-world-revolute-breakable-motor-packet.json` for the `/1`
row, plus
`BM_AvbdRigidBreakableJointStep` and
`BM_AvbdArticulatedBreakableJointStep` rows for the public free-rigid and
articulated breakable fixed point-joint paths with tracked
`avbd-rigid-breakable-joint-packet.json` and
`avbd-articulated-breakable-joint-packet.json` evidence, plus tracked
spherical break/reset packet evidence for the free-rigid, world-link
articulated, and same-multibody articulated spherical `/1` rows. The public
free-rigid-body break-force lifecycle and `avbd_rigid_breakable_joint` py-demo
are merged on
`main`; the next branch should not treat that narrow lifecycle as pending. The
current branch adds the private free-rigid versus multibody-link endpoint
classifier, `BM_AvbdRigidEndpointClassification`, and a hard fixed multibody-link
point-joint bridge into the variational articulated solve path, with
focused C++ bridge coverage plus a related
`variational_endpoint_loop_closure` public loop-closure py-demo. That py-demo
does not exercise the private AVBD point-joint config extractor. This local
slice extends simulation-entry current-pose extraction so non-topology
multibody-link fixed/revolute/prismatic point-joint entities can generate hard
AVBD configs while tree-topology joints are still skipped. It also extends the
articulated bridge so explicitly hard masked
revolute/prismatic multibody-link configs project only their active
linear/angular axes through variational loop constraints, and so explicitly
hard breakable multibody-link point-joint configs mark their source joint broken
from the projection load, with focused floating-link, fixed-breakable
mark/reset, and masked-breakable tests, including prismatic velocity-motor row
indexing and reset re-engagement.
The variational articulated projection path also now handles public one-DOF
`Velocity` actuators on revolute and prismatic topology joints as coordinate
motor targets, with focused single and coupled-chain tests. Private hard
revolute and prismatic AVBD point-joint velocity actuators on articulated
endpoints now also project one free-axis angular/linear motor row, with focused
start-of-step accumulation, revolute/prismatic command-update lifecycle,
private fixed-row break/reset, revolute/prismatic break/reset re-engagement,
and tiny positive effort-limit coverage.
Finite-stiffness private AVBD fixed
point-joint configs on articulated endpoints now also contribute compliant
variational forces through persistent per-axis finite-stiffness row state, with
focused floating-link and stiffness-ramp coverage. Public DART 7 `World`
articulated fixed/revolute/prismatic/spherical facades now create
same-multibody link-link and world-link non-topology point joints through C++
and dartpy, expose lookup/list/count helpers, and feed the simulation-entry
current-pose extractor into the variational articulated projection path.
Focused public C++ coverage now also exercises spherical linear-only
pinned-anchor behavior with free orientation and explicit link-link/world-link
anchors, bounded revolute/prismatic velocity-actuator projection,
same-multibody and world-anchored public command updates, explicit local/world
anchor binding plus same-multibody and world-anchored
fixed/revolute/prismatic off-origin anchor projection,
movable-movable same-multibody link-pair motor projection and break/reset,
same-multibody and world-anchored break/reset behavior including world-fixed
and public one-DOF motor-row re-engagement,
same-multibody/world-anchored public one-DOF motor-row break/skip and
reset/re-engagement, and
spherical linear-row break/skip/reset for same-multibody and world-link
endpoints, world-anchored
point-joint save/load rebuilding for fixed all-axis and spherical linear rows,
and
same-multibody/world-link revolute/prismatic motor save/load rebuilding through
that facade, plus same-multibody/world-link fixed/spherical/revolute/prismatic
broken-state save/load/reset persistence with focused C++ binary round-trips
that also preserve revolute/prismatic velocity-actuator state. Focused
dartpy coverage now also steps the
same-multibody/world-link revolute and prismatic public one-DOF motor
explicit-anchor break/skip and reset/re-engagement lifecycle, fixed point-joint
break/skip and reset/re-engagement for same-multibody and world-link all-axis
rows, plus the
same-multibody/world-link spherical linear-row break/skip/reset lifecycle from
Python. The local py-demo catalog now includes
`avbd_articulated_revolute_motor` and
`avbd_articulated_prismatic_motor` for same-multibody public articulated
command-update motor paths,
`avbd_articulated_motor_breakable_joint` for the same-multibody public
articulated revolute motor break/reset path,
`avbd_articulated_prismatic_pair_motor_breakable_joint` for the same-multibody
public articulated prismatic motor break/reset path,
`avbd_articulated_prismatic_motor_breakable_joint` for the world-anchored
public articulated prismatic motor break/reset path,
`avbd_articulated_world_revolute_motor_breakable_joint` for the world-anchored
public articulated revolute motor break/reset path, and
`avbd_articulated_breakable_joint` for the world-link public articulated fixed
point-joint break/reset path, with tracked
`avbd-articulated-breakable-joint-packet.json` packet evidence, plus
`avbd-articulated-fixed-pair-breakable-joint-packet.json` for the
same-multibody fixed break/reset path, plus
`avbd_articulated_spherical_breakable_joint` for the world-link public
articulated spherical break/reset path, with tracked
`avbd-articulated-spherical-breakable-joint-packet.json` packet evidence. It
also now includes
`avbd_empty_baseline`, the first runnable source-corpus baseline for
the 2D/3D empty-scene rows, with focused Python smoke coverage that checks the
reference revisions, scene indices/counts, default timestep/gravity/iteration
metadata, and `sceneEmpty` zero-count invariant. The tracked
`avbd-empty-baseline-packet.json` records the headless capture hashes and
benchmark row for that empty baseline.
The local py-demo catalog also now includes `avbd_demo2d_ground`, a static
`avbd-demo2d` Ground source-row port with one static slab, one rigid body, one
collision shape, no joints, focused Python source-row coverage, and a
`BM_AvbdDemo2dGroundStep` dashboard row. The tracked
`avbd-demo2d-ground-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; after skipping
static-only contact queries, no-op rigid dynamics stages, clean frame-cache
graph execution, and the clean no-work default step pipeline, the refreshed
packet records DART at 16.3 ns median CPU time per step versus 24.5 ns for the
native static source runner on this host, closing the narrow source row while
leaving broad ground-contact, GPU, and paper-number gates open.
The local py-demo catalog also now includes `avbd_demo2d_motor`, the first
one-DOF motor `avbd-demo2d` source-row port, with source revision, scene index,
default parameter, target-speed, and effort-limit metadata plus a focused Python
source-row invariant and `BM_AvbdDemo2dMotorStep` dashboard row. That Motor row
now has a tracked `avbd-demo2d-motor-packet.json` with headless visual capture,
DART benchmark JSON, and native `avbd-demo2d` source timing. The rigid contact
stage now skips contact queries for worlds with no collision geometry in
prepare/execute; the refreshed packet records the no-collision-geometry DART row
at 8.87 us median CPU time per step versus 1.435 us for the native source runner
on this host, so the row remains partial. A later local follow-up now reuses the
contact-stage point-joint snapshot storage across steps instead of allocating a
fresh snapshot for no-contact point-joint/motor projection. The focused motor
row tests still pass, but the exploratory benchmark ran under high load and
reported a noisy 10.04 us median, so the tracked Motor packet was not refreshed.
The local py-demo catalog now also includes `avbd_demo2d_hanging_rope`, a
matched-metadata `avbd-demo2d` Hanging Rope source-row port with 49 regular
links, one 10 m endpoint block, 49 linear-only public spherical point joints,
50 collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dHangingRopeStep` dashboard row. The tracked
`avbd-demo2d-hanging-rope-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; it records DART at
0.392 ms median CPU time per step versus 50.1 us for the native source runner
on this host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo2d_fracture`, a
matched-metadata `avbd-demo2d` Fracture source-row port with 11 chain links,
two dynamic supports, 15 falling blocks, 10 public breakable fixed joints, 29
collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dFractureStep` dashboard row. The tracked
`avbd-demo2d-fracture-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; it records DART at
51.2 us median CPU time per step versus 61.4 us for the native source runner
on this host after a refreshed same-source timing run, closing only this narrow
source-row CPU comparison.
The local py-demo catalog now also includes `avbd_demo2d_dynamic_friction`, a
matched-metadata `avbd-demo2d` Dynamic Friction source-row port with 11 sliding
boxes, source friction coefficients from 5.0 down to 0.0, a static ground, 12
collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dDynamicFrictionStep` dashboard row. The tracked
`avbd-demo2d-dynamic-friction-packet.json` records headless visual capture,
DART benchmark JSON, and native `avbd-demo2d` source timing; it records DART
at 5.73 us median CPU time per step versus 10.49 us for the native source
runner on this host, so the narrow source row is closed while broad friction,
GPU, and paper-number gates remain open.
The local py-demo catalog now also includes `avbd_demo2d_static_friction`, a
matched-metadata `avbd-demo2d` Static Friction source-row port with one rotated
static ground slab, 11 rotated dynamic boxes, uniform source friction 1.0, 12
collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dStaticFrictionStep` dashboard row. The tracked
`avbd-demo2d-static-friction-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; it records DART at
5.44 us median CPU time per step versus 14.56 us for the native source runner
on this host, so the narrow source row is closed while broad friction, GPU, and
paper-number gates remain open.
The local py-demo catalog now also includes `avbd_demo2d_pyramid`, a
matched-metadata `avbd-demo2d` Pyramid source-row port with a static ground,
210 dynamic boxes in the source pyramid layout, 211 collision shapes, focused
Python source-row coverage, and a `BM_AvbdDemo2dPyramidStep` dashboard row. The
tracked `avbd-demo2d-pyramid-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; it records DART at
0.251 ms median CPU time per step versus 2.47 ms for the 10,000-step native
source runner on this host, so the narrow Pyramid row is closed without closing
broad rigid stacking, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo2d_stack`, a
matched-metadata `avbd-demo2d` Stack source-row port with 20 vertical dynamic
boxes over static ground, 21 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo2dStackStep` dashboard row. The tracked
`avbd-demo2d-stack-packet.json` records headless visual capture, DART benchmark
JSON, and native `avbd-demo2d` source timing; it records DART at 10.37 us
median CPU time per step versus 22.46 us for the native source runner on this
host, so the narrow Stack row is closed without closing broad rigid stacking,
GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo2d_stack_ratio`, a
matched-metadata `avbd-demo2d` Stack Ratio source-row port with six
geometric-size dynamic boxes over static ground, 7 collision shapes, focused
Python source-row coverage, and a `BM_AvbdDemo2dStackRatioStep` dashboard row.
The tracked `avbd-demo2d-stack-ratio-packet.json` records headless visual
capture, DART benchmark JSON, and native `avbd-demo2d` source timing; it
records DART at 3.62 us median CPU time per step versus 8.03 us for the native
source runner on this host, so the narrow Stack Ratio row is closed without
closing broad rigid stacking, high-ratio stability, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo2d_rod`, a
matched-metadata `avbd-demo2d` Rod source-row port with 20 rigid links, one
static anchor link, 19 all-axis public fixed joints, 20 collision shapes,
focused Python source-row coverage, and a `BM_AvbdDemo2dRodStep` dashboard row.
The tracked `avbd-demo2d-rod-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; it records DART at
0.253 ms median CPU time per step versus 26.4 us for the native source runner on
this host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo2d_joint_grid`, a
matched-metadata `avbd-demo2d` Joint Grid source-row port with 625 source-shaped
rigid boxes, two static top-corner anchors, 1200 all-axis public fixed joints,
625 collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dJointGridStep` dashboard row. The tracked
`avbd-demo2d-joint-grid-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; it records DART at
15.1 ms median CPU time per step versus 7.33 ms for the native source runner on
this host, so the row remains partial. The source's 1152 diagonal
`IgnoreCollision` pairs are configured through DART's public per-pair collision
filter.
The local py-demo catalog now also includes `avbd_demo2d_soft_body`, a
matched-metadata `avbd-demo2d` Soft Body source-row port with one static ground
slab, two 15x5 dynamic rigid-box lattices, 260 finite-stiffness all-axis public
fixed joints, 224 diagonal ignored collision pairs, 151 collision shapes,
focused Python source-row coverage, and a `BM_AvbdDemo2dSoftBodyStep` dashboard
row. The tracked `avbd-demo2d-soft-body-packet.json` records headless visual
capture, DART benchmark JSON, and native `avbd-demo2d` source timing; it records
DART at 3.58 ms median CPU time per step versus 0.568 ms for the native source
runner on this host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo2d_cards`, a
matched-metadata `avbd-demo2d` Cards source-row port with one source-shaped
ground slab, 40 thin dynamic cards, 41 collision shapes, focused Python
source-row coverage, and a `BM_AvbdDemo2dCardsStep` dashboard row. The tracked
`avbd-demo2d-cards-packet.json` records headless visual capture, DART benchmark
JSON, and native `avbd-demo2d` source timing; it records DART at 0.517 ms
median CPU time per step versus 96.2 us for the native source runner on this
host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo2d_rope`, a
matched-metadata `avbd-demo2d` Rope source-row port with 20 rigid links, 19
linear-only public spherical point joints, 20 collision shapes, focused Python
source-row coverage, and a `BM_AvbdDemo2dRopeStep` dashboard row. The tracked
`avbd-demo2d-rope-packet.json` records headless visual capture, DART benchmark
JSON, and native `avbd-demo2d` source timing; it records DART at 0.133 ms
median CPU time per step versus 25.6 us for the native source runner on this
host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo2d_heavy_rope`, a
matched-metadata `avbd-demo2d` Heavy Rope source-row port with 19 regular
links, one 30 m endpoint block, 19 linear-only public spherical point joints,
20 collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dHeavyRopeStep` dashboard row. The tracked
`avbd-demo2d-heavy-rope-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo2d` source timing; after reusing per-body
row-index scratch and caching snapshot body indices, it records DART at 0.149
ms median CPU time per step versus 25.4 us for the native source runner on this
host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo2d_net`, a
matched-metadata `avbd-demo2d` Net source-row port with one static ground slab,
40 net links with static endpoints, 50 falling rigid blocks, 39 linear-only
public spherical point joints, 91 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo2dNetStep` dashboard row. The tracked
`avbd-demo2d-net-packet.json` records headless visual capture, DART benchmark
JSON, and native `avbd-demo2d` source timing; after caching snapshot body
indices and using a stable full-rank linear basis for local-anchor spherical
point-joint extraction, it records DART at 0.286 ms median CPU time per step
versus 0.232 ms for the native source runner on this host, so the row remains
partial.
The local py-demo catalog now also includes `avbd_demo3d_ground`, a
matched-metadata `avbd-demo3d` Ground source-row port with a static floor,
falling rigid box, 2 rigid bodies, 2 collision shapes, focused Python
source-row coverage that observes rigid ground contact, and a
`BM_AvbdDemo3dGroundStep` dashboard row. The tracked
`avbd-demo3d-ground-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo3d` source timing; it records DART at
5.49 us median CPU time per step versus 6.10 us for the native source runner
on this host, so the narrow Ground row is closed without closing broad
rigid-contact, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo3d_dynamic_friction`, a
matched-metadata `avbd-demo3d` Dynamic Friction source-row port with 11 sliding
boxes over a static floor, 12 collision shapes, focused Python source-row
coverage that shows high-friction boxes slow while the zero-friction box keeps
sliding, and a `BM_AvbdDemo3dDynamicFrictionStep` dashboard row. The tracked
`avbd-demo3d-dynamic-friction-packet.json` records headless visual capture,
DART benchmark JSON, and native `avbd-demo3d` source timing; it records DART at
36.85 us median CPU time per step versus 51.98 us for the native source runner
on this host, so the narrow Dynamic Friction row is closed without closing
broad contact-manifold friction persistence, stacking/friction sweeps, GPU, or
paper-number gates.
The local py-demo catalog now also includes `avbd_demo3d_static_friction`, a
matched-metadata `avbd-demo3d` Static Friction source-row port with a static
floor, inclined static ramp, 11 sliding boxes, 13 collision shapes, focused
Python source-row coverage that shows the lower-friction box sliding farther
than the high-friction box, and a `BM_AvbdDemo3dStaticFrictionStep` dashboard
row. The tracked `avbd-demo3d-static-friction-packet.json` records headless
visual capture, DART benchmark JSON, and native `avbd-demo3d` source timing; it
records DART at 48.27 us median CPU time per step versus 51.97 us for the
native source runner on this host, closing that narrow source row.
The local py-demo catalog now also includes `avbd_demo3d_pyramid`, a
matched-metadata `avbd-demo3d` Pyramid source-row port with a static ground,
136 dynamic boxes in the source triangular pile layout, 137 collision shapes,
focused Python source-row coverage that observes finite settling and contact
generation, and a `BM_AvbdDemo3dPyramidStep` dashboard row. The tracked
`avbd-demo3d-pyramid-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo3d` source timing; it records DART at
0.991 ms median CPU time per step versus 2.80 ms for the native source runner
on this host, so the narrow Pyramid row is closed without closing broad rigid
stacking, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo3d_rope`, a
matched-metadata `avbd-demo3d` Rope source-row port with 20 rigid links, 19
anchored linear-only public spherical point joints, 21 collision shapes,
focused Python source-row coverage, and a `BM_AvbdDemo3dRopeStep` dashboard
row. The tracked `avbd-demo3d-rope-packet.json` records headless visual
capture, DART benchmark JSON, and native `avbd-demo3d` source timing; after
reusing per-body row-index scratch and caching snapshot body indices, it
records DART at 0.134 ms median CPU time per step versus 35.9 us for the native
source runner on this host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo3d_heavy_rope`, a
matched-metadata `avbd-demo3d` Heavy Rope source-row port with 19 regular
links, a 5 m endpoint block, 19 anchored linear-only public spherical point
joints, the source row's intentional 0.5 m initial residual at the oversized
endpoint block, 21 collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo3dHeavyRopeStep` dashboard row. The tracked
`avbd-demo3d-heavy-rope-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo3d` source timing; after reusing per-body
row-index scratch and caching snapshot body indices, it records DART at 0.128
ms median CPU time per step versus 33.4 us for the native source runner on this
host, so the row remains partial.
The local py-demo catalog now also includes `avbd_demo3d_stack`, a
matched-metadata `avbd-demo3d` Stack source-row port with 10 vertical dynamic
boxes over static ground, 11 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo3dStackStep` dashboard row. The tracked
`avbd-demo3d-stack-packet.json` records headless visual capture, DART benchmark
JSON, and native `avbd-demo3d` source timing; it records DART at 42.1 us median
CPU time per step versus 75.7 us for the native source runner on this host, so
the narrow Stack row is closed without closing broad rigid stacking, high-ratio
stability, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo3d_stack_ratio`, a
matched-metadata `avbd-demo3d` Stack Ratio source-row port with four
geometric-size dynamic boxes over static ground, 5 collision shapes, focused
Python source-row coverage, and a `BM_AvbdDemo3dStackRatioStep` dashboard row.
The tracked `avbd-demo3d-stack-ratio-packet.json` records headless visual
capture, DART benchmark JSON, and native `avbd-demo3d` source timing; it records
DART at 18.1 us median CPU time per step versus 42.0 us for the native source
runner on this host, so the narrow Stack Ratio row is closed without closing
broad size-ratio stability, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo3d_bridge`, a
matched-metadata `avbd-demo3d` Bridge source-row port with 40 planks, 50 load
boxes, 78 paired linear-only public spherical point joints, 91 collision
shapes, focused Python source-row coverage, and a `BM_AvbdDemo3dBridgeStep`
dashboard row. The tracked `avbd-demo3d-bridge-packet.json` records headless
visual capture, DART benchmark JSON, and native `avbd-demo3d` source timing;
after reusing per-body row-index scratch and caching snapshot body indices, it
records DART at 0.746 ms median CPU time per step versus 1.20 ms for the native
source runner on this host, so the narrow Bridge row is closed without closing
broad coupled-constraint, GPU, or paper-number gates.
The local py-demo catalog now also includes `avbd_demo3d_breakable`, a
matched-metadata `avbd-demo3d` Breakable source-row port with 19 rigid bodies,
10 breakable fixed joints, 19 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo3dBreakableStep` dashboard row. The tracked
`avbd-demo3d-breakable-packet.json` records headless visual capture, DART
benchmark JSON, and native `avbd-demo3d` source timing; it records DART at
79.5 us median CPU time per step versus 112.7 us for the native source runner
on this host, closing the narrow Breakable source-row CPU gate.
Broader persistent private
articulated motor lifecycle coverage beyond the one-DOF lifecycle checks,
broad articulated fracture lifecycle/corpus coverage, and broader public
articulated World facade coverage beyond the current link-link/world-link
entrypoints, explicit anchors, spherical linear-only pinned-anchor rows,
same-multibody link-link and world-link fixed/spherical save/load rebuilding,
same-multibody/world-link revolute/prismatic motor save/load rebuilding,
same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
save/load/reset persistence,
same-multibody/world-anchored off-origin fixed/revolute/prismatic anchors and
motors,
same-multibody/world-anchored one-DOF motor break/skip, spherical linear-row
break/skip/reset including focused dartpy spherical stepping, and movable
link-pair motor/break-reset projection. A narrow
`avbd_articulated_high_ratio_chain` py-demo now covers a five-link 200:1
high mass-ratio variational-chain smoke, and
`BM_AvbdArticulatedHighRatioChainStep` adds a matching dashboard row. The
tracked `avbd-articulated-high-ratio-chain-packet.json` records an 8-frame
headless capture and 47.6 us median CPU time per step for that narrow smoke.
The C++ `PaperScaleHighRatioChainStaysFiniteAndResets` regression now adds a
50-link/50,000:1 finite/reset smoke through configured `World::step()`
solve-budget fields, and `avbd_paper_scale_high_ratio_chain` plus
`avbd-paper-scale-high-ratio-chain-packet.json` add matching paper-scale visual
and benchmark evidence. Same-hardware paper-number comparison, GPU, and broad
articulated hard-constraint coverage remain missing.
The durable AVBD corpus matrix now lives at
`docs/plans/104-vertex-block-descent-solver/avbd-demo-corpus.md`; it maps every
required `avbd-demo2d`, `avbd-demo3d`, paper, website/video, and performance
row to missing/partial DART evidence. It is a tracking artifact only, not a
parity claim.

## Immediate Next Step

Current newest local follow-up: finish verifying the narrow
`RigidBodyContactStage::prepare()` overhead cleanup from
`avbd/source-row-perf-slice` after the latest `origin/main` merge that includes
#2975. The branch currently has local benchmark smoke showing improved
Motor/Spring/Spring Ratio source rows, but this does not close any source
CPU-win, GPU, or paper-number gate. The earlier SSH fetch path failed in this
environment with
`ssh: connect to host github.com port 22: Network is unreachable`, so keep using
HTTPS when a fresh main refresh is needed here. The pre-merge backup stashes
remain present and should not be dropped without maintainer approval. The
project `.gitignore` intentionally ignores scratch JSON dumps in the PLAN-104
evidence folder while allowing curated `avbd-*-packet.json` fixtures referenced
by plan docs and packet-writer tests to be checked in.

The previous local follow-up adds dartpy articulated joint-list lifetime
coverage, proving link-link and world-link public facade handles returned from a
temporary World list keep endpoint access valid after garbage collection. The
previous local follow-up added C++ and dartpy serialization coverage
for public articulated generated-name persistence across save/load, proving
restored explicit/generated facade names plus the restored topology joint
component advance the shared joint-name counter before the next empty-name
facade is created. The earlier local follow-up added C++ and
dartpy generated-name coverage
for public non-topology articulated facades, proving empty-name facades skip an
existing `joint_001`, appear through articulated name/list lookups, and reject
duplicates. The earlier local follow-up added dartpy public articulated
`World.clear()` lifecycle coverage, verifying that world-link articulated joint
handles and name/list lookups are invalidated and that a fresh world-link
prismatic motor can be rebuilt and stepped after the reset. Another earlier local
follow-up added the C++
`VariationalIntegration.AvbdPublicArticulatedWorldFacadeClearDropsExtractedRowsAndRebuilds`
row-extraction clear/rebuild guard. Another earlier local follow-up tightened
public world-link articulated motor demo coverage for the world-prismatic and
world-revolute break/reset scenes and removed the stale world-revolute
remaining gate from the world-prismatic packet metadata. Another earlier local
follow-up added
`World.RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionReducesSlip`
and
`World.RigidBodyContactStageAvbdWarmStartedMultiTopBoxPileManifoldFrictionRunsThroughDefaultWorldStep`
for live two-lower/two-upper box-pile contact-stage/default-step friction
slip-reduction. The earlier local follow-up added
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreContactOrder` for
private stacked static/dynamic plus dynamic/dynamic contact-order replay.
Another earlier local follow-up added
`AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsIgnoreEndpointOrder` for
private multi-top box-pile endpoint-order replay. Another earlier local follow-up
added
`AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsIgnoreContactOrder` for
private multi-top box-pile contact-order replay. Earlier local follow-ups added
live endpoint-order row-identity evidence in
`AvbdContact.WorldCollideSameFeatureRowsIgnoreEndpointOrder` and
`AvbdContact.WorldCollideLiveManifoldSameFeatureRowsIgnoreEndpointOrder`, and
live sphere/mesh-face, sphere/mesh-edge, and mesh-vertex row-order evidence in
`AvbdContact.WorldCollideMeshFaceRowsIgnoreContactOrder`,
`AvbdContact.WorldCollideMeshEdgeRowsIgnoreContactOrder`, and
`AvbdContact.WorldCollideMeshVertexRowsIgnoreContactOrder`,
live mesh face/edge/vertex endpoint-order warm-start evidence in
`AvbdContact.WorldCollideMeshFaceRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideMeshEdgeRowsIgnoreEndpointOrder`, and
`AvbdContact.WorldCollideMeshVertexRowsIgnoreEndpointOrder`,
live sphere/plane friction tangent warm-start evidence in
`AvbdContact.WorldCollideFrictionRowsIgnoreContactOrder`,
live sphere/plane normal-rotation friction tangent projection evidence in
`AvbdContact.WorldCollideFrictionRowsProjectAcrossChangingPlaneNormal`,
live endpoint-swapped box-box friction tangent projection evidence in
`AvbdContact.WorldCollideLiveManifoldFrictionRowsIgnoreEndpointOrder`,
live stacked endpoint-swapped friction tangent projection evidence in
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreEndpointOrder`,
live cylinder-cap/plane row-order evidence in
`AvbdContact.WorldCollideCylinderCapRowsIgnoreContactOrder`,
live capsule-cap/plane row-order evidence in
`AvbdContact.WorldCollideCapsuleCapRowsIgnoreContactOrder`,
live cylinder-side and capsule-side row-order evidence in
`AvbdContact.WorldCollideCylinderSideRowsIgnoreContactOrder` and
`AvbdContact.WorldCollideCapsuleSideRowsIgnoreContactOrder`,
live cylinder-rim row-order evidence in
`AvbdContact.WorldCollideCylinderRimRowsIgnoreContactOrder`,
live cylinder/capsule cap/side/rim endpoint-order warm-start evidence in
`AvbdContact.WorldCollideCylinderCapRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideCylinderSideRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideCylinderRimRowsIgnoreEndpointOrder`,
`AvbdContact.WorldCollideCapsuleCapRowsIgnoreEndpointOrder`, and
`AvbdContact.WorldCollideCapsuleSideRowsIgnoreEndpointOrder`,
small-pose live box-box row-persistence evidence in
`AvbdContact.WorldCollideLiveManifoldSameFeatureRowsPersistAcrossSmallPose`,
live box-box manifold friction tangent warm-start persistence in
`AvbdContact.WorldCollideLiveManifoldFrictionRowsPersistAcrossSmallPose`,
and live stacked-box row-persistence evidence in
`AvbdContact.WorldCollideStackedManifoldsPersistRowsAcrossSmallPose`, plus
stacked-box friction warm-start persistence in
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsPersistAcrossSmallPose`,
stacked-box contact-order replay in
`AvbdContact.WorldCollideStackedManifoldsFrictionRowsIgnoreContactOrder`,
and live multi-top box-pile row-persistence evidence in
`AvbdContact.WorldCollideMultiTopBoxPileFrictionRowsPersistAcrossSmallPose`;
continue with a different remaining AVBD gap rather than repeating
generic endpoint-order, same mesh/primitive endpoint-order, same
mesh-face/mesh-edge/mesh-vertex/cylinder-cap/capsule-cap/cylinder-side/capsule-side/cylinder-rim row-order, same
sphere/plane friction warm-start mapping, same
endpoint-swapped box-box friction tangent projection, same stacked
endpoint-swapped friction tangent projection, same sphere/plane
normal-rotation friction projection, same box-box small-pose, same
box-box friction warm-start
persistence, same stacked-box nudge, same stacked-box friction persistence,
same stacked-box contact-order replay,
same multi-top stacked contact-stage/default-step friction slip coverage, or
same spanning-top or multi-top box-pile private row-persistence/contact-order/
endpoint-order or contact-stage/default-step coverage. Good next slices remain broader
rigid-contact feature persistence across realistic piles/frictional stacks, broader
articulated lifecycle/corpus coverage, or a source-demo corpus
implementation/performance row. Focused green tests here do not close the full
paper/source-demo, CPU/GPU, or published-number gates.

The latest local follow-up keeps rigid AVBD row-index layouts warm across
frames, avoids copying generated point-pair/angular rows into combined work
vectors when only one row family is active, skips endpoint row-counter hash-map
setup for one-new-row point-joint/distance-spring appends, and keeps small
point-joint/motor/distance-spring row-builder descriptor/active-row temporaries
on the stack for up to 16 candidate rows. The contact stage now also extracts
and appends point-joint and distance-spring families independently instead of
scanning both families whenever either config exists. The snapshot solve now
also clears absent row-family inventories directly instead of calling empty
contact/joint/motor/spring builders. This is aimed at source rows such as Joint
Grid that generate thousands of stable fixed-joint rows and small Motor/Spring
rows where fixed row-family setup can be a visible fraction of the step.
Focused validation ran `pixi run build-simulation-tests` and the
`test_avbd_rigid_block` rigid row-driver/contact-stage filters. Current Google
Benchmark smoke reports `BM_AvbdDemo2dMotorStep_median` at about 8.43 us,
`BM_AvbdDemo2dSpringStep_median` at about 3.98 us, and
`BM_AvbdDemo2dSpringRatioStep_median` at about 36.16 us, but this is still
overhead cleanup rather than a checked packet CPU-win gate.

The previous local follow-up skips no-op AVBD pair-constraint extraction in the
rigid contact stage when the registry has no point-joint or distance-spring
configs, and makes the split rigid-body velocity stage assemble force batches
only for advanceable bodies. Contact-only rows still build and solve their
contact snapshot, joint/spring rows still extract/append when present, and the
standalone batched SoA integration stage still uses all bodies for its
index-aligned force batch. This is source-row overhead cleanup, not a checked
CPU-win gate.

The previous local follow-up refreshed the 2D Rope, Heavy Rope, Hanging Rope, 3D
Rope, 3D Heavy Rope, and 3D Bridge source-row timing packets after the snapshot
body-index cache. The regenerated 2D packets record Rope at 0.133 ms versus
25.6 us (5.20x slower), Heavy Rope at 0.149 ms versus 25.4 us (5.85x slower),
and Hanging Rope at 0.392 ms versus 50.1 us (7.84x slower). The regenerated 3D
packets record Rope at 0.134 ms versus 35.9 us (3.75x slower), Heavy Rope at
0.128 ms versus 33.4 us (3.84x slower), and Bridge at 0.746 ms versus 1.20 ms
(1.61x faster), so only the narrow Bridge CPU row stays closed. The project
`.gitignore` intentionally ignores scratch JSON dumps in the PLAN-104 evidence
folder while allowing curated `*-packet.json` files to be checked in.

The next source-row audit checked the `avbd-demo2d` Spring/Spring Ratio scenes
against the DART row families that existed at the time. The source uses one-row
radial rigid distance springs between rigid body centers, while the earlier DART
foundations covered deformable distance springs, hard multibody distance loop
closures, and articulated per-axis compliant point-joint rows. Public distance
loop closures are hard constraints and reject free rigid-body endpoints, so they
are not a faithful source Spring substitute. The follow-up below removes the
World/API source-scene wiring blocker for rigid radial distance springs; later
packet evidence closes the packet/CPU-reference evidence gate, but not CPU
performance or GPU parity.

A previous local follow-up adds the first supported free-rigid radial spring
extraction path: `World::addRigidBodyDistanceSpring` and the dartpy
`add_rigid_body_distance_spring` binding create design-mode distance-spring
configs, the rigid contact stage extracts them into `AvbdRigidWorldDistanceSpring`
inputs, and the snapshot solve feeds them through persistent finite-stiffness
row inventory into the private serial rigid row driver. Focused coverage now
includes `AvbdRigidBlock.PointPairDistanceSpring*`,
`AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch`,
`AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows`, and
`test_simulation_world_rigid_body_distance_spring_reduces_stretch`. Local
validation for this slice ran `pixi run lint`, `pixi run build-simulation-tests`,
`build/default/cpp/Release/bin/test_avbd_rigid_block --gtest_filter='AvbdRigidBlock.RigidWorldDistanceSpringApiFeedsRadialRows:AvbdRigidBlock.RigidRowDriverReducesDistanceSpringStretch'`,
and
`PYTHONPATH=build/default/cpp/Release/python pixi run python -m pytest python/tests/unit/simulation/test_world.py -k 'distance_spring or api_exposes_python_names_only'`.
This removes the missing internal primitive/driver/API extraction foundation,
but Spring/Spring Ratio still need CPU performance resolution and GPU parity
before they can be called source ports.

The next local follow-up adds the first `avbd_demo2d_spring` and
`avbd_demo2d_spring_ratio` source-scene harnesses over that rigid radial
distance-spring API, registers both scenes in `python/examples/demos`, adds
source-row invariants in `python/tests/integration/test_demos_cycle.py`, wires
`BM_AvbdDemo2dSpringStep` and `BM_AvbdDemo2dSpringRatioStep` into the AVBD World
dashboard slice, and extends `scripts/run_avbd_demo2d_reference_timing.py` with
`--scene spring` and `--scene spring_ratio` native source timing entrypoints.
Focused validation for this follow-up ran the Spring/Spring Ratio pytest slice.
At that point, tracked packet evidence, generated CPU reference comparison,
performance resolution, and GPU parity remained missing.

The latest local follow-up adds
`scripts/write_avbd_demo2d_spring_packet.py`,
`scripts/write_avbd_demo2d_spring_ratio_packet.py`, and focused packet-writer
tests, then generates
`avbd-demo2d-spring-packet.json` and
`avbd-demo2d-spring-ratio-packet.json` from headless captures,
`BM_AvbdDemo2dSpringStep`/`BM_AvbdDemo2dSpringRatioStep` benchmark JSON, and
native source timing JSON. Refreshed packet evidence records DART about 5.09x
slower than the native Spring row and about 4.25x slower than the native Spring
Ratio row on
this host, so CPU performance resolution and GPU parity remain open.

The latest local follow-up adds `avbd_demo3d_spring` and
`avbd_demo3d_spring_ratio` source-scene harnesses over the same public radial
rigid distance-spring API, registers both scenes in `python/examples/demos`,
adds source-row invariants in `python/tests/integration/test_demos_cycle.py`,
wires `BM_AvbdDemo3dSpringStep` and `BM_AvbdDemo3dSpringRatioStep` into the
AVBD World dashboard slice, and extends
`scripts/run_avbd_demo3d_reference_timing.py` with `--scene spring` and
`--scene spring_ratio`. It also adds `scripts/write_avbd_demo3d_spring_packet.py`,
`scripts/write_avbd_demo3d_spring_ratio_packet.py`, and focused packet-writer
tests, then generates `avbd-demo3d-spring-packet.json` and
`avbd-demo3d-spring-ratio-packet.json` from headless captures,
`BM_AvbdDemo3dSpringStep`/`BM_AvbdDemo3dSpringRatioStep` benchmark JSON, and
native source timing JSON. Refreshed packet evidence records DART about 1.96x
slower than the native Spring row and about 3.70x slower than the native Spring
Ratio row on
this host, so CPU performance resolution and GPU parity remain open for both
3D rows.

An earlier local follow-up broadens
`VariationalIntegration.AvbdPublicArticulatedBreakageSurvivesSaveLoadAndReset`
and
`VariationalIntegration.AvbdPublicArticulatedWorldFixedBreakageSurvivesSaveLoadAndReset`
so same-multibody and world-link public fixed point-joint broken-state
save/load/reset persistence now uses explicit local/world anchors, verifying
the restored all-axis rows stay skipped under force and reset re-engages the
captured anchor constraints. The previous local follow-up broadened
`VariationalIntegration.AvbdPublicArticulatedRevoluteBreakageSurvivesSaveLoadAndReset`,
`VariationalIntegration.AvbdPublicArticulatedPrismaticBreakageSurvivesSaveLoadAndReset`,
`VariationalIntegration.AvbdPublicArticulatedWorldRevoluteBreakageSurvivesSaveLoadAndReset`,
and
`VariationalIntegration.AvbdPublicArticulatedWorldPrismaticBreakageSurvivesSaveLoadAndReset`
so same-multibody and world-link public one-DOF motor broken-state
save/load/reset persistence now uses explicit local/world anchors, verifying
the restored broken rows stay skipped under force and reset re-engages the
captured anchor constraints; the latest local follow-up now also preserves
non-cardinal revolute/prismatic axes through those binary round-trips and
asserts the reset-rebuilt AVBD basis columns before checking resumed free-axis
motion. The latest local follow-up also broadens the direct same-multibody and
world-link revolute/prismatic `BreakForce*` C++ lifecycle regressions so
one-DOF break/skip/reset paths use non-cardinal axes, assert the generated AVBD
basis columns, and check resumed free-axis motor progress after reset. The
previous local follow-up broadened
`test_simulation_world_articulated_point_joint_facade_exposes_link_endpoints`
with explicit-anchor same-multibody prismatic and world-link
revolute/prismatic overload coverage, so the public dartpy endpoint facade now
checks all fixed/revolute/prismatic/spherical link-link and world-link anchor
forms. The previous local follow-up broadened
`test_simulation_world_articulated_motor_breakage_reset_reengages_from_python`
and
`test_simulation_world_articulated_complementary_motor_breakage_reset_from_python`
so their same-multibody and world-link revolute/prismatic public one-DOF motor
break/skip/reset coverage uses explicit link-link and world-link anchors,
verifying broken rows can drift under external force and reset re-engages the
masked hard rows while the free-axis motor continues. The previous local
follow-up broadened
`test_simulation_world_articulated_fixed_breakage_reset_reengages_from_python`
so its same-multibody and world-link public articulated fixed point-joint
break/skip/reset coverage uses explicit link-link and world-link anchors,
verifying the all-axis rows re-engage the captured anchor pose after reset. The
previous local follow-up added
`test_simulation_world_articulated_spherical_pair_breakage_reset_reengages_from_python`,
which steps the same-multibody public articulated spherical point-joint
break/skip/reset lifecycle through dartpy with explicit link-link anchors,
verifying the linear rows re-engage the captured anchor pair after reset while
orientation remains free. The previous local follow-up added
`test_simulation_world_articulated_complementary_motor_breakage_reset_from_python`,
which steps the complementary same-multibody prismatic and world-link revolute
public one-DOF motor break/skip/reset lifecycles through dartpy, verifying the
masked hard rows and free-axis motor rows re-engage after reset. The previous
local follow-up added
`test_simulation_world_articulated_fixed_breakage_reset_reengages_from_python`,
which originally stepped same-multibody and world-link public articulated fixed
point joints through dartpy break/skip/reset and verified the all-axis rows
re-engage the captured pose after reset. This is narrow public facade lifecycle
evidence only; it does not close broader articulated facade, fracture, motor,
source-corpus, GPU, or benchmark parity gates. The previous local follow-up broadened focused
broken-state save/load coverage for public articulated facades:
`VariationalIntegration.AvbdPublicArticulatedLinkSphericalBreakageSurvivesSaveLoadAndReset`
reloads a broken same-multibody public spherical point joint, verifies the
restored linear-only rows stay skipped, and then resets it to re-engage the
loaded anchor pair while preserving free relative orientation.
`VariationalIntegration.AvbdPublicArticulatedWorldFixedBreakageSurvivesSaveLoadAndReset`
reloads a broken world-link public fixed point joint, verifies the restored
all-axis rows stay skipped, and then resets it to re-engage the explicit
world/link anchor and loaded orientation.
`VariationalIntegration.AvbdPublicArticulatedRevoluteBreakageSurvivesSaveLoadAndReset`
and
`VariationalIntegration.AvbdPublicArticulatedPrismaticBreakageSurvivesSaveLoadAndReset`
reload broken same-multibody public one-DOF velocity motors, verify the
restored hard rows stay skipped, and then reset them to re-engage the loaded
pose plus free-axis motor progression. The previous local follow-up added
`VariationalIntegration.AvbdPublicArticulatedWorldSphericalBreakageSurvivesSaveLoadAndReset`
which reloads a broken world-link public spherical point joint, verifies the restored
linear-only rows stay skipped, and then resets it to re-engage the loaded world
anchor while preserving free orientation.
`VariationalIntegration.AvbdPublicArticulatedWorldRevoluteBreakageSurvivesSaveLoadAndReset`
does the same for a world-link public revolute velocity motor, including
free-axis motor progress after reset. The earlier local follow-up added
`VariationalIntegration.AvbdPublicArticulatedBreakageSurvivesSaveLoadAndReset`
for a same-multibody public fixed joint after it has broken, verifying the
restored joint stays broken and skipped, then resets it and checks the AVBD rows
re-engage the explicit link-link anchor pair and loaded orientation.
`VariationalIntegration.AvbdPublicArticulatedWorldPrismaticBreakageSurvivesSaveLoadAndReset`
does the same for a world-link public prismatic velocity motor, including
free-axis motor progress after reset. This is narrow public facade persistence
evidence only; it does not close broader articulated facade, fracture, motor,
source-corpus, GPU, or benchmark parity gates. An earlier local follow-up
broadens focused C++ save/load coverage for public
articulated facades:
`VariationalIntegration.AvbdPublicArticulatedFixedJointFacadeSurvivesSaveLoad`
and
`VariationalIntegration.AvbdPublicArticulatedWorldFixedJointFacadeSurvivesSaveLoad`
reload public same-multibody link-link and world-link fixed point joints, check
preserved break-force/facade state, and verify simulation entry rebuilds the
private AVBD all-axis rows that hold the serialized anchors and orientation.
`VariationalIntegration.AvbdPublicArticulatedWorldSphericalJointFacadeSurvivesSaveLoad`
and
`VariationalIntegration.AvbdPublicArticulatedLinkSphericalJointFacadeSurvivesSaveLoad`
reload public world-link and same-multibody link-link joints, check preserved
break-force/facade state, and verify simulation entry rebuilds the private AVBD
linear-only rows that pin the serialized anchors while leaving orientation
free. `VariationalIntegration.AvbdPublicArticulatedRevoluteJointFacadeSurvivesSaveLoad`,
`VariationalIntegration.AvbdPublicArticulatedWorldRevoluteJointFacadeSurvivesSaveLoad`,
`VariationalIntegration.AvbdPublicArticulatedPrismaticJointFacadeSurvivesSaveLoad`,
and
`VariationalIntegration.AvbdPublicArticulatedWorldPrismaticJointFacadeSurvivesSaveLoad`
also reload same-multibody and world-link revolute/prismatic velocity-motor
facades and verify the private hard rows plus free-axis motor row rebuild from
serialized actuator, anchor, effort-limit, and break-force state; the
same-multibody/world-link revolute cases and the world-link prismatic case now
preserve non-cardinal axes through the binary round-trip and assert the rebuilt
AVBD basis columns after simulation entry. This is
narrow public facade persistence evidence only; it does not close broader
articulated facade, fracture, motor, source-corpus, GPU, or benchmark parity
gates. The earlier
local follow-up adds `avbd_articulated_high_ratio_chain`, a
five-link 200:1 high mass-ratio articulated variational-chain smoke py-demo,
plus focused integration coverage for swing motion and reset, and adds
`BM_AvbdArticulatedHighRatioChainStep` as the corresponding narrow dashboard
row. The current local follow-up adds the matching
`avbd-articulated-high-ratio-chain-packet.json` visual/benchmark packet. This
is narrow paper-gap evidence only; the later
`PaperScaleHighRatioChainStaysFiniteAndResets` smoke covers a 50-link/50,000:1
finite/reset envelope through configured `World::step()` solve-budget fields,
but it does not close the paper benchmark-packet, GPU, or broad
articulated-chain gates. The
previous local follow-up adds
`avbd_rigid_spherical_breakable_joint`, a public free-rigid
spherical point-joint break/reset py-demo, plus focused integration coverage for
break, release, reset, captured anchor re-engagement, and orientation remaining
free, now backed by `avbd-rigid-spherical-breakable-joint-packet.json` packet
evidence. The earlier local follow-up extends
`avbd_rigid_breakable_joint`, the public free-rigid fixed point-joint break-force
demo, with reset/re-engagement controls, tracked
`avbd-rigid-breakable-joint-packet.json` packet evidence, and focused
integration coverage for break, release, reset, and captured fixed-pose
re-engagement. The earlier local follow-up adds
`avbd_articulated_fixed_pair_breakable_joint`, a same-multibody public
articulated fixed break/reset py-demo, plus tracked
`avbd-articulated-fixed-pair-breakable-joint-packet.json` packet evidence and
focused integration coverage for break, release, reset, and captured
relative-pose re-engagement. The previous local follow-up added
`avbd_articulated_spherical_pair_breakable_joint`, a same-multibody public
articulated spherical break/reset py-demo, plus focused integration coverage
for break, release, reset, anchor re-engagement, and orientation remaining
free, now backed by
`avbd-articulated-spherical-pair-breakable-joint-packet.json` packet evidence.
The earlier local follow-up added
`avbd_articulated_prismatic_pair_motor_breakable_joint`, a same-multibody
public articulated prismatic motor break/reset py-demo, plus focused
integration coverage for break, lateral release, reset, and re-engagement.
The prior local follow-up added
`avbd_articulated_world_revolute_motor_breakable_joint`, a world-anchored
public articulated revolute motor break/reset py-demo, plus focused integration
coverage for the matching world-anchored revolute lifecycle. An earlier
follow-up added
`avbd_articulated_prismatic_motor_breakable_joint`, a world-anchored public
articulated prismatic motor break/reset py-demo, plus focused integration
coverage for the matching prismatic lifecycle. These are narrow
CPU/user-visible evidence only; they do not close broader articulated motor or
fracture lifecycle, fracture-corpus, GPU, or paper/source-runner performance
gates.

The latest local follow-up refreshed the `avbd_demo2d_fracture` packet from a
fresh same-source DART/native timing pair: it records DART at 51.2 us/step
versus 61.4 us/step for the native source runner, about 1.20x faster. This
closes only the narrow Fracture source-row CPU comparison; broad fracture
corpus, GPU, and paper-number gates remain open. Existing local work also keeps
a mechanical rigid-contact-stage allocation cleanup: `RigidBodyContactStage` reuses its
extracted AVBD point-joint input scratch, AVBD solve-row scratch, and per-body
row-index scratch, and the
rigid contact snapshot solve passes already-predicted inertial targets by span
while preserving the old copy fallback for direct solves that have no predicted
targets. It also closes a source-row mismatch from the pinned native
`sceneFracture`: the source solver's broadphase skips body pairs connected by a
live force, so DART collision queries now suppress live public rigid-body joint
endpoint pairs while allowing broken AVBD joints to become collidable again.
The contact stage now also replaces its old duplicate prepare-time collision
query with collision-shape-count constraint prewarm, so execute-time contact
discovery remains the only authoritative rigid contact query. The live-pair set
now reuses `CollisionQueryCache` scratch, no longer needs a `Name` component
view, skips empty ignored-pair lookups, and the contact-stage preflight scan now
combines collision-geometry and dynamic-endpoint checks. Focused `test_world`
coverage checks warmed filtered queries do not allocate global heap. The
Fracture py-demo metadata, source timing runner, C++ benchmark counter, and
packet writer now record the 10 joint-connected collision pairs without adding
persistent `setCollisionPairIgnored()` state. Focused `test_avbd_rigid_block`,
`test_world`, AVBD `test_variational_integration`, and Fracture Python packet
slices pass, but benchmark attempts in
`/tmp/avbd_inertial_span_benchmark.json` and
`/tmp/avbd_joint_input_reuse_benchmark.json`, same-turn diagnostic runs in
`/tmp/avbd_fracture_solve_scratch_final.json` and
`/tmp/avbd_solve_scratch_adjacent_rows.json`,
`/tmp/avbd_fracture_joint_filter_diagnostic.json`, and
`/tmp/avbd_fracture_joint_filter_cached_pairs.json`, plus the later
`/tmp/avbd_demo2d_fracture_after_prepare_skip.json`, and a later unsaved
same-host diagnostic after the query-filter micro-cleanup that reported about
139 us median CPU time with host load above 22, all remain non-packet
diagnostics. The refreshed packet uses the fresh same-turn
`/tmp/avbd-demo2d-fracture-refresh.json` DART benchmark and
`/tmp/avbd-demo2d-fracture-reference-refresh.json` native timing. A
local-anchor point-joint extraction experiment was reverted because same-turn
diagnostic timing did not support keeping the larger input payload. A narrower
later follow-up keeps only an `anchorsAreLocal` flag on the existing input
payloads and uses it only when the private contact-stage path asks extraction to
skip world-anchor reconstruction.

Latest continuation: `AvbdBreakablePointJointConfigResetReengagesFixedRows`
and `AvbdBreakableChildPointJointConfigResetReengagesFixedRows` now save the
broken private explicit fixed point-joint configs, reload them, check restored
entity/config state, and reset the loaded worlds so this private fixed-row
lifecycle is covered across binary save/load for parent-link and child-link
endpoint polarity rather than only in-memory.
`AvbdBreakableSphericalPointJointConfigSurvivesSaveLoadAndReset`
and
`AvbdBreakableSphericalChildPointJointConfigSurvivesSaveLoadAndReset` now add
the matching private explicit world-link spherical paths, preserving the
linear-only row mask, endpoint shape, and broken state across binary save/load
for parent-link and child-link endpoint polarity before reset re-engages only
the anchor rows while orientation remains free.
`AvbdBreakableRevoluteVelocityParentPointJointConfigSurvivesSaveLoadAndReset`
and
`AvbdBreakablePrismaticVelocityParentPointJointConfigSurvivesSaveLoadAndReset`
now add direct private one-DOF velocity-motor persistence with the multibody
link on the parent side of the world-link joint, covering the opposite endpoint
polarity from the child-link save/load/reset paths; the existing child-link
direct one-DOF save/load/reset paths now also assert restored endpoint shape.
`AvbdBreakableRevoluteVelocityParentPointJointConfigResetReengagesMotorRows`
and
`AvbdBreakablePrismaticVelocityParentPointJointConfigResetReengagesMotorRows`
now add direct private in-memory parent-endpoint reset/re-engagement coverage
for the one-DOF motor rows, proving the opposite free-axis sign is rebuilt
without relying on the binary round trip.
`AvbdRevolutePointJointVelocityActuatorParentEndpointRespectsTinyTorqueLimit`
and
`AvbdPrismaticPointJointVelocityActuatorParentEndpointRespectsTinyForceLimit`
now add the matching direct private parent-endpoint tiny effort-limit coverage,
proving bounded free-axis motor rows stay capped for the opposite world-link
endpoint polarity.
`AvbdRevolutePointJointVelocityActuatorParentEndpointFollowsCommandUpdates`
and
`AvbdPrismaticPointJointVelocityActuatorParentEndpointFollowsCommandUpdates`
now add direct private parent-endpoint command-update coverage for the same
one-DOF motor rows, proving command changes rebuild the opposite-sign
world-link motor targets instead of reusing stale child-endpoint polarity.
`AvbdRevolutePointJointVelocityActuatorParentEndpointFollowsNonCardinalCommandUpdates`
and
`AvbdPrismaticPointJointVelocityActuatorParentEndpointFollowsNonCardinalCommandUpdates`
now add the matching direct private parent-endpoint command-update coverage for
generated non-cardinal free-axis bases, proving the opposite-sign parent
polarity is not an artifact of the cardinal X/Z rows.
`AvbdRevolutePointJointVelocityActuatorFollowsNonCardinalCommandUpdates` and
`AvbdPrismaticPointJointVelocityActuatorFollowsNonCardinalCommandUpdates` now
add direct private child-endpoint command-update coverage for generated
non-cardinal free-axis bases, completing the child/parent world-link
non-cardinal command-update polarity slice.
`AvbdRevolutePointJointVelocityActuatorRespectsTinyTorqueLimitOnNonCardinalAxis`,
`AvbdRevolutePointJointVelocityActuatorParentEndpointRespectsTinyTorqueLimitOnNonCardinalAxis`,
`AvbdPrismaticPointJointVelocityActuatorRespectsTinyForceLimitOnNonCardinalAxis`,
and
`AvbdPrismaticPointJointVelocityActuatorParentEndpointRespectsTinyForceLimitOnNonCardinalAxis`
now add direct private child/parent finite-effort coverage for generated
non-cardinal free-axis bases, proving tiny positive effort limits stay capped
without relying on the cardinal X/Z rows.

Continue the next bounded AVBD contact/friction or rigid-block step:
broader private articulated motor lifecycle coverage, broader articulated
fracture lifecycle/corpus coverage beyond the current private/public
same-multibody, world-link, explicit-anchor, off-origin, binary-save/load/reset,
spherical linear-row including direct private world-link fixed/spherical
parent/child save/load/reset, and
one-DOF motor regression set including direct revolute/prismatic parent/child
command-update including child/parent non-cardinal axis-basis coverage,
save/load, tiny-limit including child/parent non-cardinal axis-basis coverage,
and in-memory reset endpoint polarity,
or public articulated World facade/demo-packet coverage for combinations not
yet promoted into the corpus,
or optimize the measured 2D Soft Body source-row CPU gap.
Other open rigid-row gaps remain: full narrow-phase feature extraction for
remaining realistic manifolds, broader
rigid-contact feature persistence beyond the current
box/sphere/cylinder/capsule/plane/mesh known/unknown shape-frame
local-transform inference, endpoint-A/B explicit-shape local-point evidence,
actual narrow-phase primitive-feature evidence, and same-feature sphere/plane
replay plus sphere/mesh-face, sphere/mesh-edge, and mesh-vertex replay, live cylinder-cap/plane row-order evidence, live box-box row-order,
endpoint-order, and small-pose persistence evidence plus the first live stacked
static/dynamic and dynamic/dynamic manifold row-persistence evidence,
or the first implementation row from the AVBD
corpus matrix are the preferred row-family gaps now that
private dynamic/rigid contact feature IDs, canonical two-endpoint row keys, and
normal/friction row descriptor helpers plus private rigid contact/friction
point-pair constructors, paired friction-cone helpers, and a private serial
rigid row driver plus private rigid contact-manifold row builder and
World-contact snapshot/solve/writeback helpers plus a combined private wrapper,
first internal contact-stage activation, box-feature/pair-scoped row identity,
live sphere/plane and box-box manifold endpoint-order row identity,
deterministic same-feature contact row ordering including actual `World::collide()`
sphere/plane plus sphere/mesh-face, sphere/mesh-edge, and mesh-vertex replay, live
cylinder-cap/plane and capsule-cap/plane row-order coverage, live
sphere/plane friction tangent row warm-start mapping, live cylinder-side and
capsule-side row-order coverage, live cylinder-rim row-order coverage, live endpoint-swapped
box-box friction tangent projection, live stacked endpoint-swapped friction
tangent projection, live sphere/plane normal-rotation friction tangent
projection, and live box-box manifold box-face feature coverage,
private cylinder side/cap/rim and capsule side/top-cap/bottom-cap endpoint
features, known/unknown shape-frame feature mapping with endpoint-A/B
explicit-shape local-point coverage, actual narrow-phase
sphere/cylinder/capsule/plane/mesh primitive-feature coverage, and private
point-joint linear/angular/combined rows with step-start previous constraint
values and private World snapshot/step append/solve/apply coverage plus
fixed-joint ECS extraction through the step helper and an explicit current-pose
rigid-body fixed-joint config bridge plus simulation-entry
config initialization for opt-in rigid bodies, masked private point-joint row
generation for constrained linear/angular axes, and named private
revolute/prismatic point-joint configs with arbitrary joint-axis bases plus
private rigid-body ECS current-pose extraction for those one-DOF joint
entities, and public free rigid-body revolute/prismatic facades with dartpy
bindings, stubs, tests, and py-demo coverage, plus fixed-joint/contact,
free-rigid revolute/prismatic-motor, articulated revolute/prismatic-motor, and
free-rigid
plus world-link articulated fixed and spherical breakable-joint AVBD py-demos,
the empty
source-corpus baseline py-demo, the first non-empty `avbd-demo2d` Ground,
Motor, Hanging Rope, Fracture, Dynamic Friction, Static Friction, Pyramid, and
Stack source-row py-demos and packets, the `avbd-demo2d` Rope and Heavy Rope
source-row py-demos and packets, the `avbd-demo2d` Stack Ratio, Rod, and Joint
Grid source-row py-demos and packets, the `avbd-demo3d` Ground and Dynamic
Friction source-row py-demos and packets, the `avbd-demo3d` Static Friction
source-row py-demo and packet, the `avbd-demo3d` Pyramid source-row py-demo and
packet, the `avbd-demo3d` Rope and Heavy Rope source-row py-demos and packets,
the `avbd-demo3d` Stack, Stack Ratio, and Soft Body source-row py-demos and packets, the
`avbd-demo3d` Bridge source-row py-demo and packet,
the `avbd-demo3d` Breakable source-row py-demo and packet, narrow
free-rigid and
articulated motor, active breakable-motor, and breakable fixed point-joint
benchmark rows, and the
narrow free-rigid-body break-force
lifecycle plus fixed/revolute/prismatic/spherical binary save/load broken-state
reset coverage and free-rigid/articulated design-mode AVBD point-joint
stiffness facade save/load coverage, private free-rigid versus multibody-link endpoint
classification, endpoint benchmark row, focused hard fixed articulated endpoint
bridge tests, related public variational endpoint loop-closure py-demo, and
public articulated fixed/revolute/prismatic/spherical link-link and world-link
point-joint facades exist, with focused spherical linear-only pinned-anchor and
break/skip/reset coverage plus world-anchored one-DOF tiny effort-limit
coverage.
The 2D Ground packet records the static DART public World row about 1.51x
faster than the native source runner on this host after skipping static-only
contact queries, no-op rigid dynamics stages, clean frame-cache graph
execution, and the clean no-work default step pipeline with a cheap scratch
reset, closing that narrow source row while broad ground-contact, GPU, and
paper-number gates remain open.
The Motor packet records the no-collision-geometry DART public World row still
about 6.18x slower than the native source runner on this host after skipping
contact queries in contact-stage prepare/execute, so that CPU gate also remains
open. The
Hanging Rope packet
records the DART public World row about 7.84x slower than the native source
runner on this host, so that CPU gate also remains open. The Fracture packet
records the DART public World row about 1.20x faster than the native source
runner on this host after a refreshed same-source timing run, closing only that
narrow source row. The 3D Breakable packet
records the DART public World row about 1.42x faster than the native source
runner on this host, closing that narrow source row. The 3D Ground packet
records the DART public World row about 1.11x faster than the native source
runner on this host, and the 3D Dynamic Friction packet records the DART public
World row about 1.41x faster than the native source runner on this host,
closing only those narrow source rows. The 2D Stack packet records the DART
public World row about 2.17x faster than the native source runner on this host,
closing only that narrow source row. The 2D Stack Ratio packet records the DART
public World row about 2.22x faster than the native source runner on this host,
closing only that narrow source row. The 2D Static Friction packet records the
DART public World row about 2.68x faster than the native source runner on this
host, closing only that narrow source row. The 2D Pyramid packet records the
DART public World row about 9.84x faster than the 10,000-step native source
runner on this host, closing only that narrow source row. The 3D Static Friction
packet records the DART public World row about 1.08x faster than the native
source runner on this host, closing only that narrow source row. The 3D Pyramid
packet records the DART public World row about 2.83x faster than the native
source runner on this host, closing only that narrow source row. The 2D Rope
packet records the DART public World row about 5.20x slower than the native
source runner on this host, so that CPU gate also remains open. The 2D Rod
packet records the DART public World row about 9.58x slower than the native
source runner on this host, so that CPU gate also remains open. The 2D Heavy
Rope packet records the DART public World row about 5.85x slower than the native
source runner on this host, so that CPU gate also remains open. The 3D Rope
packet records the DART public World row about 3.75x slower than the native
source runner on this host, so that CPU gate also remains open. The 2D Joint
Grid packet records the DART public World row about 2.06x slower than the native
source runner on this host after reusing per-body row-index scratch and caching
snapshot body indices, with its source diagonal ignore-collision pairs
configured through the public per-pair filter. The 2D Cards packet records the
DART public World row about 5.38x slower than the native source runner on this
host, so that CPU gate also remains open. The 3D Heavy Rope packet records the DART
public World row about 3.84x slower than the native source
runner on this host, so that CPU gate also remains open. The Stack and Stack
Ratio packets record the DART public World rows about 1.80x and 2.32x faster
than the native source
runners on this host, closing only those narrow source rows. The Bridge packet
records the DART public World row about 1.61x faster than the native source
runner on this host, closing only that narrow source row. The next source-corpus
step should either optimize a measured source-runner CPU gap or record why
another ready row is the smaller correctness/performance gate.
Keep the supported envelope narrow and preserve fallback coverage for topology
mixes,
damping/acceleration, parallel solves, and unsupported requested row
combinations.

## Context That Would Be Lost

- AVBD belongs under PLAN-104 because it is the hard-constraint continuation of
  the VBD solver family, not a duplicate roadmap entry.
- Completion must preserve the full paper bar: all algorithms/features, CPU and
  GPU implementations, all paper/site/video/demo scenes, and benchmark JSON
  proving DART beats reference and paper numbers.
- The 2D reference demo source has 19 scenes; the 3D source has 14 scenes. Both
  are clarity references, not optimized baselines, but DART still must beat them
  before making CPU performance claims.
- The current AVBD code intentionally covers scalar row equations, row
  inventory warm starting, one CPU contact-normal kernel, one CPU hard
  attachment kernel, one CPU finite-stiffness spring kernel, and narrow
  internal World opt-ins for static contact-normal rows, hard point-attachment
  rows, and finite-stiffness spring rows, including a combined serial
  mass-spring row solve for those three families. Pure-tet World scenes can now
  use the tet-material finite-stiffness row path and, when requested, combine it
  with AVBD self-contact normal/friction rows. Supported static-contact
  mass-spring World scenes generate bounded friction-tangent rows with pairwise
  static/dynamic switching, and static box obstacle row keys now distinguish
  faces, edges, and corners. Persisting static half-space friction rows project
  their decayed tangential dual across changing smooth tangent bases, and
  persisting self-contact friction rows project their generalized tangential dual
  across changing 12D tangent stencils. Supported serial mass-spring
  self-contact scenes generate AVBD self-contact normal rows and matching
  self-contact friction rows when material friction is positive. A combined
  static/self-contact friction regression now guards both row families in one
  supported World solve. The first private rigid foundation now has a 6-DOF
  block accumulator, world-frame quaternion tangent update, inertia term, block
  solve, scalar rigid point-attachment row, and two-body point-pair row
  stamping. Point-pair rows also carry a scalar offset with private
  rigid-contact normal and bounded friction tangent constructors plus paired
  tangent static/dynamic switching and circular Coulomb-cone projection for
  future contact-manifold generation, and a private serial rigid row driver can
  sweep point attachments, contact-normal point pairs, and paired friction
  tangent rows. A private rigid contact-manifold row builder now converts active
  contact points with stable endpoint feature IDs into warm-started normal and
  paired tangent rows for that driver. A private World-contact snapshot helper
  now converts rigid-body `World::collide()` output into manifold-point inputs
  for that builder, runs them through the private serial rigid row solve, and
  writes dynamic rigid-body state back to the ECS through a combined private
  wrapper in focused tests. The first internal contact-stage activation now
  predicts rigid inertial targets from current velocities, solves supported
  opted-in contacts through the private 6-DOF rows, and projects the solved
  displacement back into rigid velocity. This does not imply
  hard-contact/friction completeness, full contact-manifold friction
  persistence, broad dynamic/rigid feature persistence, rigid/articulated
  joints, rigid/soft coupling, or GPU parity. Private dynamic/rigid contact
  identity helpers now pack contact feature kind/index IDs, canonicalize
  two-endpoint contact row keys, derive private box face/edge/corner endpoint
  feature IDs and cylinder side/cap/rim plus capsule side/top-cap/bottom-cap
  endpoint features for rigid snapshots, map known and unknown-index contacts
  through body and collision-shape local transforms before feature coding, use
  narrow-phase shape-local contact points for explicit endpoint-A/B compound
  shape-index feature coding, cover actual `World::collide()`
  sphere/cylinder/capsule/plane/mesh contacts through primitive feature IDs, and
  include unique plane compound matches that derive shape-scoped face IDs and
  mesh compound matches that derive
  face/edge/vertex feature IDs, scope row ordinals per canonical endpoint pair,
  assign same-feature contact rows from deterministic canonical-local point
  ordering with actual `World::collide()` sphere/plane plus sphere/mesh-face,
  sphere/mesh-edge, and mesh-vertex replay, live cylinder-cap/plane and capsule-cap/plane row-order
  coverage, live sphere/plane friction tangent row warm-start mapping, and live
  box-box manifold box-face feature coverage, and create
  normal/friction/joint-linear/joint-angular row descriptors,
  with private point-joint rows now seeding step-start previous
  constraint values and participating in the private World snapshot/solve/apply
  wrapper and combined step helper from world-space point-joint inputs, plus
  private fixed-joint ECS extraction through the step helper and internal
  contact-stage velocity projection for rigid-body-linked joint entities with
  or without active contacts. Named private revolute and prismatic point-joint
  configs now preserve arbitrary joint-axis bases and leave one rotational or
  translational axis free, and private rigid-body ECS revolute/prismatic joint
  entities now initialize and extract through the same current-pose bridge.
  Public rigid-body revolute/prismatic velocity actuators now extract to AVBD
  angular/linear motor rows, and the private extractor now classifies
  free-rigid endpoints separately from multibody links with benchmark and
  focused fixed/masked/breakable bridge test coverage. Public one-DOF
  articulated velocity actuators now also project
  through the variational path, and private hard revolute/prismatic AVBD
  point-joint velocity actuators now project one articulated motor row along
  their free axis with revolute/prismatic command-update, private fixed-row
  break/reset, revolute/prismatic break/reset re-engagement, and tiny positive
  effort-limit coverage. Finite-stiffness
  private AVBD fixed point-joint
  configs on articulated endpoints now contribute compliant variational forces
  through persistent stiffness-ramped rows.
  Non-topology multibody-link fixed/revolute/prismatic point-joint entities now
  also initialize hard private AVBD configs from the simulation-entry current
  pose, and public same-multibody/world-link articulated
  fixed/revolute/prismatic/spherical facades now feed that extractor through
  C++/dartpy with focused spherical linear-only pinned-anchor behavior including
  explicit link-link/world-link anchors, same-multibody link-link and
  world-link fixed/spherical save/load rebuilding,
  bounded revolute/prismatic
  velocity-motor, same-multibody and
  world-anchored command updates and tiny effort-limit behavior, explicit
  local/world anchor projection including same-multibody and world-anchored
  off-origin fixed/revolute/prismatic anchors and revolute/prismatic motors,
  same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
  save/load/reset persistence including explicit-anchor fixed and one-DOF motor
  rows,
  movable-movable same-multibody motor projection
  and fixed break/reset, private fixed-row and revolute/prismatic break/reset
  re-engagement, same-multibody and world-anchored public break/reset
  re-engagement including world-fixed and one-DOF motor rows,
  same-multibody/world-anchored public one-DOF motor break/skip and
  reset/re-engagement including focused dartpy stepping coverage for
  same-multibody/world-link revolute and prismatic explicit-anchor cases, spherical
  linear-row break/skip/reset,
  world-anchor and spherical facade tests, and the
  `avbd_articulated_revolute_motor`,
  `avbd_articulated_prismatic_motor`, and
  `avbd_articulated_breakable_joint`,
  `avbd_articulated_fixed_pair_breakable_joint`, and
  `avbd_articulated_spherical_breakable_joint` and
  `avbd_articulated_spherical_pair_breakable_joint` py-demos, plus
  `avbd-articulated-breakable-joint-packet.json`,
  `avbd-articulated-fixed-pair-breakable-joint-packet.json`,
  `avbd-articulated-spherical-breakable-joint-packet.json`,
  `avbd-articulated-spherical-pair-breakable-joint-packet.json`,
  `avbd_articulated_high_ratio_chain`,
  `BM_AvbdArticulatedHighRatioChainStep`,
  `avbd-articulated-high-ratio-chain-packet.json`,
  `avbd_empty_baseline`,
  `avbd_demo2d_ground`, `avbd_demo2d_motor`, `avbd_demo2d_hanging_rope`,
  `avbd_demo2d_fracture`,
  `avbd_demo2d_dynamic_friction`, `avbd_demo2d_static_friction`,
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
  `avbd_demo3d_pyramid`, `avbd_demo3d_rope`, `avbd_demo3d_heavy_rope`,
  `avbd_demo3d_bridge`, and `avbd_demo3d_breakable`, narrow CPU dashboard
  benchmark rows for the matched-metadata empty source-corpus baseline, the
  matching empty-row visual/benchmark packet, the first non-empty
  2D Motor/Hanging Rope/Fracture/Dynamic Friction/Static Friction/Cards/Stack/Stack Ratio/Rod/Joint Grid/Rope/Heavy Rope/Net
  and 3D Ground/Dynamic Friction/Static Friction/Pyramid/Rope/Heavy
  Rope/Bridge/Breakable source rows and packets,
  public motor, and breakable fixed point-joint paths including private
  explicit fixed binary save/load/reset.
  Full
  narrow-phase feature extraction, broad
  motor/fracture lifecycle and benchmark coverage beyond these fixed/world-link
  reset rows, spherical linear-row fracture checks, and public one-DOF motor
  break/skip checks, broader public articulated
  World facade coverage, paper/source-demo py-demos, and CPU/GPU parity are not
  solved yet.

## How to Resume

```bash
git status --short --branch
pixi run build-simulation-tests
build/default/cpp/Release/bin/test_variational_integration --gtest_filter='VariationalIntegration.Avbd*'
build/default/cpp/Release/bin/test_variational_integration --gtest_filter='VariationalIntegration.VelocityActuator*'
build/default/cpp/Release/bin/test_avbd_constraint
build/default/cpp/Release/bin/test_avbd_rigid_block
build/default/cpp/Release/bin/test_boxed_lcp_contact --gtest_filter='AvbdContact.*:BoxedLcpContact.MethodSelectorReflectsConstruction'
build/default/cpp/Release/bin/test_vbd_combined_descent --gtest_filter='VbdCombinedDescent.AvbdSelfContact*'
build/default/cpp/Release/bin/test_vbd_attachment
build/default/cpp/Release/bin/test_vbd_finite_stiffness
build/default/cpp/Release/bin/test_vbd_contact --gtest_filter='VbdContact.Avbd*'
build/default/cpp/Release/bin/test_vbd_world_solver --gtest_filter='VbdWorldSolver.AvbdTetRowsCombineSelfContactFrictionRows:VbdWorldSolver.AvbdContactAndSelfContactFrictionRowsCombine:VbdWorldSolver.AvbdSelfContactFrictionRowsReduceTangentialMotion:VbdWorldSolver.AvbdSelfContactNormalRowsIncludeFrictionTangentRows:VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain:VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial:VbdWorldSolver.AvbdFiniteStiffnessRowsFallbackForUnsupportedEnvelopes:VbdWorldSolver.AvbdContactNormalRowsFallbackForFriction:VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody:VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness:VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode:VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact'
```

Then continue Phase A1 from
[`README.md#immediate-next-steps`](README.md#immediate-next-steps).
