# LCP Solver Interface And Demos — Dev Task

## 2026-06-13 Current Continuation - Demo README Benchmark Command Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `f3594d980f3 Guard LCP benchmark packet coverage` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `f3594d980f3 Guard LCP benchmark packet coverage`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by sixty-one commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Demo README benchmark-command status:

- `python/tests/unit/test_py_demo_panels.py` now checks that the LCP demo README
  benchmark smoke command stays synchronized with the `_BENCHMARK_COMMAND`
  metadata in `python/examples/demos/scenes/lcp_physics.py`.
- `python/examples/demos/README.md` now points readers at the scene metadata's
  derived `representative_benchmark_command` for the full representative packet
  filter instead of duplicating the long generated filter list in prose.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, generated profile
  CSVs, generated evidence CSVs, and demo scene runtime behavior were not
  intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'readme_commands_match_scene_metadata or lcp_physics_exposes_solver_manifest_and_benchmark_metadata'`
  passed with 2 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `pixi run lint` passed, including LCP solver roster and generated AI command
  checks.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed end to end. The documentation phase still
  emitted the known four `dartpy._world_render_bridge` autodoc warnings, but
  the full gate passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused demo-panel metadata test, `pixi run lint`, `git diff --check`, and
   any broader gate warranted by the final diff, then
   commit the focused README/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Benchmark Coverage Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `9bcb6d09ff5 Expose LCP CUDA benchmark packets` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `9bcb6d09ff5 Expose LCP CUDA benchmark packets`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by sixty commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Benchmark coverage-guard status:

- `scripts/check_lcp_solver_roster.py` now enforces the previous uncovered
  benchmark audit as a permanent invariant: every parsed demo benchmark filter
  must match a registered `BM_LCP_COMPARE`/`BM_Lcp*` base, and every registered
  base must be covered by at least one demo-side representative filter prefix.
- `python/tests/unit/test_check_lcp_solver_roster.py` now includes negative
  coverage for the drift path where a registered benchmark base is not covered
  by any demo benchmark filter.
- This hardens the previous CUDA benchmark-packet checkpoint, where the local
  audit reported `uncovered 0`, without adding more benchmark packets.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, generated profile
  CSVs, generated evidence CSVs, and demo scene metadata were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or uncovered'`
  passed with 3 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 30 tests.
- `pixi run lint` passed, including the LCP solver roster check.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed on this Linux host with an NVIDIA RTX
  5000 Ada visible via `nvidia-smi`; the CUDA gate included lint, build, C++
  unit tests, simulation tests, Python tests, documentation, CUDA-labeled
  tests, and CUDA benchmark smoke wrappers. The documentation phase still
  emitted the existing 4 autodoc warnings for `dartpy._world_render_bridge`,
  but the full gate passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full roster unit test, roster check, `pixi run lint`, `git diff --check`,
   `pixi run build`, and any broader gate warranted by the final diff, then
   commit the focused script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - CUDA Benchmark Packets

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `1a85a2ee3ae Expose LCP boxed world-step benchmark packet` as the
   latest completed local tip before this checkpoint. If this section is
   committed, inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `1a85a2ee3ae Expose LCP boxed world-step benchmark packet`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-nine commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

CUDA benchmark-packet status:

- `python/examples/demos/scenes/lcp_physics.py` now exposes dedicated
  `cuda_batch_scale` and `cuda_contact_batch` benchmark packets.
- `cuda_batch_scale` covers the registered CUDA Jacobi/PGS standalone and
  grouped batch prefixes for standard, boxed, and friction-index families.
- `cuda_contact_batch` covers the registered CUDA Jacobi/PGS world-contact,
  dense-box, stack, articulated, and mixed contact grouped batch prefixes.
- The demo representative benchmark command now derives coverage for every
  registered `BM_LCP_COMPARE`/`BM_Lcp*` base parsed by
  `scripts/check_lcp_solver_roster.py`; the local uncovered audit reported
  `uncovered 0`.
- `python/tests/unit/test_py_demo_panels.py` now checks that both CUDA packets
  remain in the demo metadata with their exact filter prefixes.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads representative CUDA benchmark-filter tokens from the
  demo metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, generated profile
  CSVs, and generated evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python - <<'PY' ...` local uncovered benchmark
  audit passed with `uncovered 0`.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and CUDA workflow
  checks.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed on this Linux host with an NVIDIA RTX
  5000 Ada visible via `nvidia-smi`; the CUDA gate included lint, build, C++
  unit tests, simulation tests, Python tests, documentation, CUDA-labeled tests,
  and CUDA benchmark smoke wrappers.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Boxed World-Step Benchmark Packet

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `b66e3c59a3d Expose LCP validation benchmark packet` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `b66e3c59a3d Expose LCP validation benchmark packet`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-eight commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Boxed world-step benchmark-packet status:

- `python/examples/demos/scenes/lcp_physics.py` now exposes a dedicated
  `world_contact_step` benchmark packet for `BM_LcpWorldSeparatedStep_BoxedLcp`
  and `BM_LcpWorldBoxStep_BoxedLcp`.
- This makes the representative benchmark command cover the C++ boxed-LCP
  world step rows for separated-contact sphere packets and dense box-contact
  packets, instead of only covering assembled world-contact rows.
- `python/tests/unit/test_py_demo_panels.py` now checks that the packet remains
  in the demo metadata with both world-step filters.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads these world-step benchmark-filter tokens from the demo
  metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Validation Benchmark Packet

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `ca63ca866d3 Expose LCP Newton warm-start batch filters` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `ca63ca866d3 Expose LCP Newton warm-start batch filters`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-seven commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Validation benchmark-packet status:

- `python/examples/demos/scenes/lcp_physics.py` now exposes a dedicated
  `validation_friction_index` benchmark packet for
  `BM_LcpValidation_Serial_FrictionIndex` and
  `BM_LcpValidation_Threaded_FrictionIndex`.
- This makes the representative benchmark command cover the C++ validation
  benchmark family that measures serial and threaded residual/complementarity
  validation for friction-index packets.
- `python/tests/unit/test_py_demo_panels.py` now checks that the packet remains
  in the demo metadata with both validation filters.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads these validation benchmark-filter tokens from the demo
  metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Newton Warm-Start Batch Filters

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `34fb7b74010 Expose LCP near-singular batch filters` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `34fb7b74010 Expose LCP near-singular batch filters`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-six commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Newton warm-start benchmark-filter status:

- `python/examples/demos/scenes/lcp_physics.py` now includes
  `BM_LcpNewtonWarmStartBatchSerial` and
  `BM_LcpNewtonWarmStartBatchParallel` in the `active_set_transition`
  benchmark packet alongside `BM_LcpActiveSetTransition` and
  `BM_LcpNewtonWarmStart`.
- This makes the representative benchmark command call out the C++ Newton
  warm-start batch benchmark family that backs active-set warm-start pressure,
  instead of relying on the broader `BM_LcpNewtonWarmStart` prefix alone.
- `python/tests/unit/test_py_demo_panels.py` now checks that the packet remains
  in the demo metadata with all four filters.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads these Newton warm-start benchmark-filter tokens from
  the demo metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Near-Singular Batch Benchmark Filters

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `5c4444ed18c Expose LCP mild conditioning benchmark packet` as the
   latest completed local tip before this checkpoint. If this section is
   committed, inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `5c4444ed18c Expose LCP mild conditioning benchmark packet`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-five commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Near-singular benchmark-filter status:

- `python/examples/demos/scenes/lcp_physics.py` now includes
  `BM_LcpNearSingularBatchSerial` and `BM_LcpNearSingularBatchParallel` in the
  `near_singular` benchmark packet alongside `BM_LcpNearSingular`.
- This makes the representative benchmark command cover the C++ near-singular
  batch benchmark family that backs the existing `near_singular_standard`
  representative solver case and related conditioning pressure.
- `python/tests/unit/test_py_demo_panels.py` now checks that the packet remains
  in the demo metadata with all three filters.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads these near-singular benchmark-filter tokens from the
  demo metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Mild Ill-Conditioned Benchmark Packet

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `8f738daf0cf Point LCP parameter rows at threading benchmarks` as the
   latest completed local tip before this checkpoint. If this section is
   committed, inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `8f738daf0cf Point LCP parameter rows at threading benchmarks`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-four commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Mild ill-conditioned benchmark-packet status:

- `python/examples/demos/scenes/lcp_physics.py` now exposes a dedicated
  `mild_ill_conditioned` benchmark packet for `BM_LcpMildIllConditioned`,
  `BM_LcpMildIllConditionedBatchSerial`, and
  `BM_LcpMildIllConditionedBatchParallel`.
- This makes the representative benchmark command cover the C++ conditioning
  benchmark family that backs the existing `ill_conditioned_standard`
  representative solver case.
- `python/tests/unit/test_py_demo_panels.py` now checks that the packet remains
  in the demo metadata with all three filters.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads these mild-ill-conditioned benchmark-filter tokens from
  the demo metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Threaded Parameter Benchmark Filters

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `b34a44bab66 Show LCP manifest support counts separately` as the
   latest completed local tip before this checkpoint. If this section is
   committed, inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `b34a44bab66 Show LCP manifest support counts separately`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-three commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Threaded parameter benchmark-filter status:

- `python/examples/demos/scenes/lcp_physics.py` now points the advanced
  `Jacobi` parameter row at `BM_LcpJacobiSolverThreading` instead of the
  unrelated `BM_LcpPgsRelaxationSweep`.
- The `RedBlackGaussSeidel` and `BlockedJacobi` parameter rows now include
  their threaded benchmark filters alongside the existing relaxation and block
  partition filters.
- The representative `solver_parameter_sweeps` benchmark packet now includes
  the Jacobi, red-black Gauss-Seidel, and blocked-Jacobi threading filters, so
  the generated representative command covers worker-thread tuning rows.
- `python/tests/unit/test_py_demo_panels.py` now checks the corrected advanced
  parameter benchmark filters and representative packet filter.
- `python/tests/unit/test_check_lcp_solver_roster.py` now checks that the LCP
  roster checker reads these threaded benchmark-filter tokens from the demo
  metadata.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, full roster unit test, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Solver Manifest Summary Counts

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `2a63696d3cc Render LCP coverage labels from support flags` as the
   latest completed local tip before this checkpoint. If this section is
   committed, inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `2a63696d3cc Render LCP coverage labels from support flags`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-two commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Solver manifest summary-count status:

- `python/examples/demos/scenes/lcp_physics.py` now renders the LCP panel
  solver-manifest summary as separate standard, boxed, and friction-index
  counts instead of the ambiguous `boxed/findex-capable` count.
- The summary now shows `24 solvers, 23 standard, 15 boxed, 16 friction-index`,
  which matches the exported manifest counts and the friction-index-only
  `Staggering` support label.
- `python/tests/unit/test_py_demo_panels.py` now checks the rendered panel
  summary text directly.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, roster check, `pixi run lint`,
   `git diff --check`, `pixi run build`, and any broader gate warranted by the
   final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Solver Manifest Coverage Labels

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0c072e73fd7 Use manifest names for LCP parameter rows` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0c072e73fd7 Use manifest names for LCP parameter rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty-one commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Solver manifest coverage-label status:

- `python/examples/demos/scenes/lcp_physics.py` now renders solver-manifest
  coverage labels from the actual standard/boxed/findex support booleans
  instead of treating every non-boxed+findex solver as standard-only.
- The LCP panel now labels the manifest-only friction-index solver
  `Staggering` as `findex only; std/boxed delegate`.
- `python/tests/unit/test_py_demo_panels.py` now exercises the rendered panel
  table and checks the all-surface, standard-only, and friction-index-only
  coverage labels.
- Solver implementations, solver support predicates, benchmark registration
  code, profile artifacts, bindings, stubs, public APIs, and generated
  profile/evidence CSVs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, roster check, `pixi run lint`,
   `git diff --check`, `pixi run build`, and any broader gate warranted by the
   final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Parameter Row Manifest Names

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0603e78891e Guard LCP demo benchmark filters` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0603e78891e Guard LCP demo benchmark filters`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifty commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Parameter row manifest-name status:

- `python/examples/demos/scenes/lcp_physics.py` now labels advanced parameter
  rows with manifest solver names for `BGS`, `NNCG`, and `MPRGP` instead of
  Python class-spelling variants.
- `python/tests/unit/test_py_demo_panels.py` now asserts that advanced
  parameter rows are a subset of the manifest-derived solver name set, so
  user-facing table labels stay aligned with the solver roster.
- Solver implementations, benchmark registration code, profile artifacts,
  bindings, stubs, public APIs, and generated profile/evidence CSVs were not
  intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or lcp_physics_updates_live_metrics_headlessly'`
  passed with 2 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, roster check, `pixi run lint`,
   `git diff --check`, `pixi run build`, and any broader gate warranted by the
   final diff, then commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Benchmark Filter Roster Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `6dce6771507 Document LCP partial profile help` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `6dce6771507 Document LCP partial profile help`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-nine commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Benchmark filter roster status:

- `scripts/check_lcp_solver_roster.py` now extracts the benchmark-filter tokens
  exposed by `python/examples/demos/scenes/lcp_physics.py`, including the demo
  smoke filter, representative benchmark packet filters, and profile smoke
  filter.
- The roster checker compares those tokens against benchmark-name prefixes
  parsed from `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`, so stale
  user-facing demo filters fail the existing LCP roster lint gate.
- `python/tests/unit/test_check_lcp_solver_roster.py` now covers current token
  extraction and rejects a synthetic stale demo benchmark token.
- Benchmark registration code, profile artifacts, solver implementations,
  bindings, stubs, public APIs, and generated profile/evidence CSVs were not
  intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q -k 'benchmark_filter or surfaces_match'`
  passed with 3 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 29 tests.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   roster unit test, roster check, `pixi run lint`, `git diff --check`,
   `pixi run build`, and any broader gate warranted by the final diff, then
   commit the focused checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Partial Profile Help Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `9b395763617 Guard LCP README profile commands` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `9b395763617 Guard LCP README profile commands`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-eight commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Partial profile help status:

- `scripts/lcp_performance_profile.py` now exposes `build_arg_parser()` so the
  profile generator's CLI help contract can be unit-tested without shelling out
  through `main()`.
- The `--allow-partial` help text now explains that partial benchmark JSON only
  relaxes incomplete native solver coverage and that partial runs must use a
  scratch `--output` directory because checked
  `docs/background/lcp/figures` output is refused.
- `python/tests/unit/test_lcp_performance_profile.py` now verifies that
  `--allow-partial`, the scratch-output wording, and the checked output
  directory appear in the generated parser help.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and C++ solver code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q -k 'partial_scratch_output or help_documents_partial_scratch_output'`
  passed with 2 tests.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 68 tests.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --help`
  showed the `--allow-partial` scratch-output warning in the generated CLI
  help.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   profile-generator unit test, roster check, `pixi run lint`,
   `git diff --check`, `pixi run build`, and any broader gate warranted by the
   final diff, then commit the focused script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - README Profile Command Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `7f9f2c6f519 Expose LCP profile smoke command` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `7f9f2c6f519 Expose LCP profile smoke command`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-seven commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

README profile command status:

- `python/examples/demos/README.md` now matches the LCP scene metadata's full
  performance-profile refresh command, including `--benchmark-timeout 900`.
- `python/tests/unit/test_py_demo_panels.py` now parses the LCP profile command
  fenced blocks in `python/examples/demos/README.md` and compares them against
  the scene-owned `_PERFORMANCE_PROFILE_REFRESH_COMMAND` and
  `_PERFORMANCE_PROFILE_SMOKE_COMMAND` constants. Future README drift in either
  the checked refresh command or scratch-output smoke command now fails the
  demo-panel regression.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and C++ solver code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_readme_profile_commands_match_scene_metadata or lcp_physics_exposes_solver_manifest_and_benchmark_metadata or profile_evidence_schema'`
  passed with 4 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 78 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, roster check, `pixi run lint`, `git diff --check`,
   `pixi run build`, and any broader gate warranted by the final diff, then
   commit the focused README/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile Smoke Command

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `ca1f794c514 Guard partial LCP profile output` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `ca1f794c514 Guard partial LCP profile output`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-six commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile smoke command status:

- `python/examples/demos/scenes/lcp_physics.py` now exposes
  `performance_profile_smoke_command` alongside the checked
  `performance_profile_refresh_command`. The smoke command uses
  `--allow-partial`, the profile-compatible
  `BM_LcpCompare/Standard/Dantzig/12` filter, and scratch outputs under
  `build/lcp_profile_smoke`.
- The LCP panel renders `profile smoke: ...` in the Performance profiles
  section, so the same safe scratch command is visible through the user-facing
  demo metadata.
- `python/examples/demos/README.md` now separates the full checked refresh
  command from the quick profile-pipeline smoke command. The smoke example does
  not target `docs/background/lcp/figures`.
- `python/tests/unit/test_py_demo_panels.py` verifies the smoke command exact
  text, checks for `--allow-partial`, checks the scratch output target, checks
  that the checked profile artifact directory is absent, and verifies the
  rendered panel text.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and C++ solver code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_exposes_solver_manifest_and_benchmark_metadata or profile_evidence_schema'`
  passed with 3 tests.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --run --allow-partial --benchmark-filter BM_LcpCompare/Standard/Dantzig/12 --benchmark-min-time 0.01 --cache build/lcp_profile_smoke.json --output build/lcp_profile_smoke --benchmark-timeout 120`
  exited successfully, warned about incomplete native solver coverage, cached
  results to `build/lcp_profile_smoke.json`, and wrote scratch artifacts under
  `build/lcp_profile_smoke/`.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 77 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, direct scratch profile smoke command, roster
   check, `pixi run lint`, `git diff --check`, `pixi run build`, and any
   broader gate warranted by the final diff, then commit the focused
   demo/docs/test change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Partial Profile Output Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `58630f0e824 Guard rendered LCP evidence schema rows` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `58630f0e824 Guard rendered LCP evidence schema rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-five commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Partial profile output guard status:

- `scripts/lcp_performance_profile.py` now refuses `--allow-partial` when the
  output target resolves to the checked `docs/background/lcp/figures` artifact
  directory. Partial smoke profile runs must choose an explicit scratch output
  such as `build/lcp_profile_smoke`.
- The guard runs before benchmark/cache loading or output-directory creation,
  so a partial smoke invocation cannot overwrite the durable
  `performance_profile_evidence.csv` or `performance_profile_<category>.csv`
  artifacts.
- `python/tests/unit/test_lcp_performance_profile.py` covers the rejected
  checked-output target and the accepted scratch-output path.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and C++ solver code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 67 tests.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --allow-partial --cache build/lcp_partial_guard_missing.json`
  exited with status 1 before output work and printed the checked-output
  refusal.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   profile-generator unit test, direct partial-output CLI check, roster check,
   `pixi run lint`, `git diff --check`, `pixi run build`, and any broader gate
   warranted by the final diff, then commit the focused script/test/docs
   change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Rendered Evidence Schema Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `cd873ba119c Cover LCP demo evidence schema fields` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `cd873ba119c Cover LCP demo evidence schema fields`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-four commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Rendered demo schema guard status:

- `python/tests/unit/test_py_demo_panels.py` now verifies that
  `lcp_physics.build().info["performance_profile_evidence_schema_rows"]`
  exposes exactly the user-facing schema rows owned by
  `python/examples/demos/scenes/lcp_physics.py`.
- The rendered-panel regression now checks every schema row's field list and
  meaning through the fake panel builder instead of checking a copied subset.
  Future schema rows such as `category, solver, problem_size` or `time_ns`
  cannot silently disappear from the demo panel without failing the test.
- Product demo code, generated profile/evidence CSVs, bindings, stubs, solver
  predicates, public APIs, benchmark generator code, and C++ solver code were
  not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'profile_evidence_schema or exposes_solver_manifest_and_benchmark_metadata'`
  passed with 3 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 77 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, roster check, `pixi run lint`, `git diff --check`,
   `pixi run build`, and any broader gate warranted by the final diff, then
   commit the focused test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Demo Evidence Schema Coverage

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `142e164ac90 Guard LCP plot CSV export` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `142e164ac90 Guard LCP plot CSV export`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-three commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Demo evidence schema status:

- `python/examples/demos/scenes/lcp_physics.py` now documents every required
  `performance_profile_evidence.csv` column in its user-facing performance
  profile evidence schema table, including the base identity fields
  `category`, `solver`, `problem_size`, and the timing field `time_ns`.
- The schema rows are ordered to flatten to
  `_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS`, so the demo-facing
  explanation stays tied to the actual CSV contract.
- `python/tests/unit/test_py_demo_panels.py` now verifies that the schema rows
  cover exactly the required columns with no duplicates or extras.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and C++ solver code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k 'lcp_physics_profile_evidence_schema_rows_cover_required_columns or lcp_physics_exposes_solver_manifest_and_benchmark_metadata'`
  passed with 2 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with 76 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed. The CUDA gate included lint, build,
  unit, simulation, Python, documentation, CUDA simulation tests, and CUDA
  benchmark smoke; the final report was 7/7 tests passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   full demo-panel unit test, roster check, `pixi run lint`,
   `git diff --check`, `pixi run build`, and `pixi run -e cuda test-all`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Plot CSV Export Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `da1d03ffe24 Guard LCP profile CSV export` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `da1d03ffe24 Guard LCP profile CSV export`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-two commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Plot CSV export status:

- `scripts/lcp_performance_profile.py` now writes the sibling
  `performance_profile_<category>.csv` artifact before optional Matplotlib
  plotting. A plotting-enabled environment can no longer refresh profile PNGs
  while skipping the CSV artifact and its validation path.
- The same preflight validation used by direct CSV exports now runs before any
  plotting attempt, so malformed profile data cannot leave behind a plotted
  profile artifact.
- `python/tests/unit/test_lcp_performance_profile.py` covers the
  Matplotlib-enabled path with a fake plotter and verifies invalid profile data
  fails before a PNG or CSV is written.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 65 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including the LCP solver roster and generated AI
  command checks.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed. The CUDA gate included lint, build,
  unit, simulation, Python, documentation, CUDA simulation tests, and CUDA
  benchmark smoke; the final report was 7/7 tests passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused profile tests, roster check, `pixi run lint`, `git diff --check`,
   `pixi run build`, and `pixi run -e cuda test-all`, then commit the focused
   generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile CSV Preflight Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `26182605777 Guard LCP empty evidence export` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `26182605777 Guard LCP empty evidence export`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty-one commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile CSV preflight status:

- `scripts/lcp_performance_profile.py` now validates performance-profile CSV
  rows before opening `performance_profile_<category>.csv` outputs. Direct
  writer calls can no longer leave header-only or partially written profile
  artifacts that the roster checker rejects later.
- The profile writer now rejects missing solver columns, empty tau rows,
  invalid or non-increasing tau values, mismatched profile vector lengths, and
  non-finite or out-of-range profile values before writing.
- `python/tests/unit/test_lcp_performance_profile.py` covers valid profile CSV
  output plus each preflight rejection path.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 63 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed, including the CUDA smoke path.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, `git diff --check`,
   `pixi run build`, and `pixi run -e cuda test-all`, then commit the focused
   generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Generator Empty Evidence Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `6d1d0434e04 Guard LCP semantic evidence rows` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `6d1d0434e04 Guard LCP semantic evidence rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by forty commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Generator empty-evidence status:

- `scripts/lcp_performance_profile.py` now rejects attempts to write
  `performance_profile_evidence.csv` with no evidence rows. The writer builds
  and validates rows before opening the output file, so a header-only artifact
  is not left behind.
- This aligns direct writer behavior with downstream roster and py-demo
  evidence consumers, which already reject empty performance-profile evidence.
- `python/tests/unit/test_lcp_performance_profile.py` covers both the new
  empty-export rejection and the schema-header check through a valid evidence
  row.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 56 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed, including the CUDA smoke path.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, `git diff --check`,
   `pixi run build`, and `pixi run -e cuda test-all`, then commit the focused
   generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Generator Semantic Evidence Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `6a33317bb05 Guard LCP evidence writer rows` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `6a33317bb05 Guard LCP evidence writer rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-nine commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Generator semantic evidence-writer status:

- `scripts/lcp_performance_profile.py` now validates current-schema semantic
  evidence counters immediately before writing `performance_profile_evidence.csv`,
  so direct writer calls cannot emit rows whose solver identity, solver family,
  solver support, native form support, or problem-type counters would be
  rejected by the roster checker or demo evidence summary later.
- The writer guard remains row-scoped: it does not require full native solver
  coverage, but any row it writes must describe a known solver that is native
  to the requested problem category and whose current-schema counters match the
  manifest.
- `python/tests/unit/test_lcp_performance_profile.py` covers invalid direct
  writer inputs for solver identity schema/index, solver family one-hot,
  solver support counters, and problem-type counters.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 55 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed, including the CUDA smoke path.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, `git diff --check`,
   `pixi run build`, and `pixi run -e cuda test-all`, then commit the focused
   generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Generator Evidence Writer Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `2928d09bd76 Guard LCP generator dimensions` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `2928d09bd76 Guard LCP generator dimensions`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-eight commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Generator evidence-writer status:

- `scripts/lcp_performance_profile.py` now validates current-schema evidence
  rows immediately before writing `performance_profile_evidence.csv`, so direct
  writer calls cannot emit invalid numeric rows that the roster checker or demo
  evidence summary would reject later.
- The writer guard checks positive problem and LCP dimensions, FrictionIndex
  contact counts, optional non-friction contact counts, positive timing,
  `contract_ok`, non-negative integer iterations, and non-negative finite
  quality metrics.
- `python/tests/unit/test_lcp_performance_profile.py` covers invalid direct
  writer inputs for LCP dimension, optional contact count, time, contract
  status, iterations, and quality metrics.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 49 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed, including the CUDA smoke path.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, `git diff --check`,
   `pixi run build`, and `pixi run -e cuda test-all`, then commit the focused
   generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Generator LCP Dimension Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `ce521817bcf Guard LCP generator contact counts` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `ce521817bcf Guard LCP generator contact counts`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-seven commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Generator LCP-dimension status:

- `scripts/lcp_performance_profile.py` now rejects present `lcp_dimension`
  counters that are non-finite, fractional, zero, or negative before
  profile/evidence CSV generation.
- Historical rows that omit `lcp_dimension` remain accepted by coverage
  validation; rows that provide the current-schema counter must provide a
  finite, exactly integral, positive dimension.
- `python/tests/unit/test_lcp_performance_profile.py` covers zero, negative,
  fractional, infinite, and NaN `lcp_dimension` values that previously passed
  generator coverage validation.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 42 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.
- `pixi run build` passed.
- `pixi run -e cuda test-all` passed, including the CUDA smoke path.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, `git diff --check`,
   `pixi run build`, and `pixi run -e cuda test-all`, then commit the focused
   generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Generator Contact Count Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0925a74253b Guard LCP generator problem sizes` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0925a74253b Guard LCP generator problem sizes`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-six commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Generator optional-contact-count status:

- `scripts/lcp_performance_profile.py` now rejects invalid optional
  `contact_count` values on non-friction profile rows before profile/evidence
  CSV generation.
- The guard follows the checked evidence rule: Standard and Boxed rows may omit
  `contact_count`, but a present value must be finite, exactly integral, and
  non-negative.
- `python/tests/unit/test_lcp_performance_profile.py` covers negative,
  fractional, infinite, and NaN optional contact counts that previously passed
  generator coverage validation.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 37 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Generator Positive Size Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `07c1654453f Guard LCP roster nonfinite counters` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `07c1654453f Guard LCP roster nonfinite counters`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-five commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Generator positive-size status:

- `scripts/lcp_performance_profile.py` now rejects benchmark JSON rows whose
  benchmark-name problem size is zero or negative before profile/evidence CSV
  generation.
- The guard covers both the current
  `BM_LcpCompare/<category>/<solver>/<problem-size>` naming schema and the
  historical `BM_LcpCompare_<solver>_<category>/<problem-size>` cache schema.
- `python/tests/unit/test_lcp_performance_profile.py` covers zero and negative
  problem sizes that previously passed when the dimension counter matched the
  invalid name.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 33 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Roster Non-Finite Counter Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `3289fb847b6 Guard LCP demo fractional counters` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `3289fb847b6 Guard LCP demo fractional counters`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-four commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Roster non-finite counter status:

- `scripts/check_lcp_solver_roster.py` now rejects non-finite integer counters
  before rounding, so invalid evidence rows produce aggregated roster evidence
  errors instead of parser exceptions.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers a non-finite
  `problem_size` evidence counter.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, benchmark generator code, and Python demo code were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 27 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_check_lcp_solver_roster.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Demo Fractional Counter Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `ba85115c4c2 Normalize LCP support counters` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `ba85115c4c2 Normalize LCP support counters`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-three commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Demo fractional counter status:

- `python/examples/demos/scenes/lcp_physics.py` now rejects fractional
  integer counters in performance-profile evidence rows instead of truncating
  them with `int(...)`.
- `python/tests/unit/test_py_demo_panels.py` covers fractional problem sizes,
  iterations, solver identity schema versions, and solver manifest indices.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and benchmark generator code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q -k lcp_physics_profile_summary`
  passed with 32 tests and 43 deselected.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused demo evidence tests above, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Support Counter Normalization

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `cb2e1ddb68b Guard LCP quality metric rows` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `cb2e1ddb68b Guard LCP quality metric rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-two commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Support counter normalization status:

- `scripts/lcp_performance_profile.py` now parses
  `solver_supports_problem`, form-support counters, and problem-type counters
  with the integer counter helper instead of raw numeric comparisons.
- Current-schema support counters now reject invalid rows with evidence errors
  instead of raising type errors when benchmark JSON stores parseable numeric
  strings.
- `python/tests/unit/test_lcp_performance_profile.py` covers both a rejected
  string `solver_supports_problem=0` row and an accepted current-schema row
  whose counters arrive as numeric strings.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Quality Metric Row Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `52620aa9b83 Normalize LCP profile ratio inputs` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `52620aa9b83 Normalize LCP profile ratio inputs`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty-one commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Quality metric row status:

- `scripts/lcp_performance_profile.py` now rejects invalid `iterations`,
  `residual`, `complementarity`, and `bound_violation` evidence before writing
  the performance-profile evidence CSV.
- Integer counter parsing now rejects non-finite values cleanly before rounding,
  so non-finite counters produce evidence errors instead of parser exceptions.
- `python/tests/unit/test_lcp_performance_profile.py` covers negative,
  fractional, and non-finite quality metric/counter rows.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Ratio Value Normalization

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `443350bf895 Guard LCP invalid timing rows` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `443350bf895 Guard LCP invalid timing rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirty commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Ratio value normalization status:

- `scripts/lcp_performance_profile.py` now uses the same parsed evidence
  helpers for ratio generation that coverage validation uses for `contract_ok`
  and `time_ns`, so coverage-accepted numeric values cannot later fail ratio
  computation as raw strings.
- `python/tests/unit/test_lcp_performance_profile.py` covers a checked
  numeric-string timing and contract row through ratio generation.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Invalid Timing Row Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `8a7c4df24ce Guard LCP failed contract rows` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `8a7c4df24ce Guard LCP failed contract rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-nine commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Invalid timing row status:

- `scripts/lcp_performance_profile.py` now rejects parsed benchmark rows with
  missing, zero, negative, or non-finite timing before those rows can feed
  performance-ratio math.
- `python/tests/unit/test_lcp_performance_profile.py` covers zero, negative,
  and NaN timing rows.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Failed Contract Row Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0b570dc2ea3 Guard LCP benchmark duplicate rows` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0b570dc2ea3 Guard LCP benchmark duplicate rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-eight commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Failed contract row status:

- `scripts/lcp_performance_profile.py` now rejects parsed benchmark rows whose
  `contract_ok` counter is missing, non-integer, or not `1` before those rows
  can count as solver coverage or feed profile generation.
- `python/tests/unit/test_lcp_performance_profile.py` covers the failed
  contract row rejection path.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Duplicate Benchmark Row Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `be935a3936f Guard LCP profile CSV rows` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `be935a3936f Guard LCP profile CSV rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-seven commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Duplicate benchmark row status:

- `scripts/lcp_performance_profile.py` now rejects duplicate parsed benchmark
  rows for the same `(category, solver, problem_size)` before the profile
  result map can silently overwrite one run with another.
- `python/tests/unit/test_lcp_performance_profile.py` covers duplicate
  `BM_LcpCompare/<category>/<solver>/<problem-size>` rows.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_lcp_performance_profile.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile CSV Row Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `79f213659ac Guard LCP evidence duplicate rows` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `79f213659ac Guard LCP evidence duplicate rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-six commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile CSV row status:

- `scripts/check_lcp_solver_roster.py` now validates
  `performance_profile_standard.csv`, `performance_profile_boxed.csv`, and
  `performance_profile_frictionindex.csv` data rows, not only their solver
  headers.
- The checker rejects empty profile CSVs, malformed row widths, invalid or
  non-increasing tau values, and solver profile values outside `[0, 1]`.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers invalid profile
  row shapes, tau values, monotonicity, and profile values.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, public
  APIs, and Python demo code were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `python/tests/unit/test_check_lcp_solver_roster.py`,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Duplicate Evidence Row Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0e6b67546e9 Guard strict LCP evidence export` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0e6b67546e9 Guard strict LCP evidence export`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-five commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Duplicate evidence row status:

- `scripts/check_lcp_solver_roster.py` now rejects repeated
  `(category, solver, problem_size)` rows in
  `performance_profile_evidence.csv` before duplicate evidence can satisfy
  coverage while double-counting demo summary rows.
- `python/examples/demos/scenes/lcp_physics.py` now applies the same duplicate
  row guard while building the performance-profile evidence summary table.
- Focused checker and py-demo tests cover duplicate-row rejection.
- Generated profile/evidence CSVs, bindings, stubs, solver predicates, and
  public APIs were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py::test_lcp_profile_evidence_rejects_duplicate_row -q`
  passed.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_duplicate_evidence_rows -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused checker test, focused py-demo test,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused checker/demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Strict Evidence Export Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `52c5a236d9a Guard LCP evidence header duplicates` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `52c5a236d9a Guard LCP evidence header duplicates`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-four commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Strict evidence export status:

- `scripts/lcp_performance_profile.py` still parses historical cached
  benchmark rows for compatibility, but `save_profile_evidence_csv()` now
  refuses to write rows missing required current-schema evidence fields.
- Blank `contact_count` remains allowed for non-FrictionIndex profiles; other
  required evidence fields must be present before export.
- `python/tests/unit/test_lcp_performance_profile.py` covers the rejected
  historical-row export path.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --cache build/lcp_profile_full.json --output docs/background/lcp/figures`
  ran the benchmark path because the requested cache was not present or usable,
  then refreshed the checked profile/evidence CSV artifacts under
  `docs/background/lcp/figures/`.
- Bindings, stubs, solver predicates, and public APIs were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --cache build/lcp_profile_full.json --output docs/background/lcp/figures`
  passed after running `BM_LCP_COMPARE`, caching
  `build/lcp_profile_full.json`, and refreshing
  `performance_profile_evidence.csv`, `performance_profile_standard.csv`,
  `performance_profile_boxed.csv`, and
  `performance_profile_frictionindex.csv`.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused generator tests, profile generation/export,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused generator/test/docs/artifact
   change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Evidence Header Duplicate Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `5d2c772867d Guard LCP profile header duplicates` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `5d2c772867d Guard LCP profile header duplicates`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-three commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Evidence-header duplicate-guard status:

- `scripts/check_lcp_solver_roster.py` now rejects duplicate columns in
  `performance_profile_evidence.csv` before `csv.DictReader` can collapse
  ambiguous names.
- `python/examples/demos/scenes/lcp_physics.py` now applies the same duplicate
  evidence-column guard before building the demo evidence summary table.
- `python/tests/unit/test_check_lcp_solver_roster.py` and
  `python/tests/unit/test_py_demo_panels.py` cover duplicate evidence-column
  regressions.
- Generated profile CSVs, bindings, stubs, solver predicates, public APIs, and
  performance timings were not intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 21 tests.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_duplicate_evidence_columns -q`
  passed.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, the duplicate evidence-column demo test,
   `scripts/check_lcp_solver_roster.py`, `pixi run lint`, and
   `git diff --check`, then commit the focused checker/demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile Header Duplicate Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `d9d9650f875 Test LCP profile header guards` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `d9d9650f875 Test LCP profile header guards`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-two commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile-header duplicate-guard status:

- `scripts/check_lcp_solver_roster.py` now rejects duplicate solver columns in
  checked LCP performance-profile CSV headers before doing set-based
  missing/unknown/non-native comparisons.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers the duplicate
  solver-column regression alongside the existing malformed, unknown, and
  non-native header cases.
- Generated profile CSVs, bindings, stubs, solver predicates, demo runtime
  behavior, public APIs, and performance timings were not intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 20 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile Header Guard Tests

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `f711df5312a Guard exact LCP stub classes` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `f711df5312a Guard exact LCP stub classes`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty-one commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile-header guard-test status:

- `python/tests/unit/test_check_lcp_solver_roster.py` now covers
  `check_performance_profile_headers()` rejecting malformed CSV headers,
  unknown solver columns, and non-native solver columns.
- The production roster checker, generated profile CSVs, bindings, stubs,
  solver predicates, demo runtime behavior, public APIs, and performance
  timings were not intentionally changed in this test-only slice.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 19 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Exact Stub Class Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `453ee4669fc Guard LCP math stub exports` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `453ee4669fc Guard LCP math stub exports`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twenty commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Exact stub class-guard status:

- `scripts/check_lcp_solver_roster.py` now rejects extra dartpy-bound LCP
  solver classes that are not present in the C++ manifest, instead of checking
  only that manifest classes are bound.
- The roster check also rejects extra `python/stubs/dartpy/math.pyi`
  subclasses of `LcpSolver` that are not present in the manifest. The broader
  `__all__` lists still use inclusion checks because they also contain
  non-solver exports.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers both extra bound
  solver classes and extra math-stub solver subclasses.
- No generated stubs, bindings, solver predicates, demo runtime behavior,
  public APIs, checked profile CSVs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 16 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Math Stub Export Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `4a0a5c4292f Tighten LCP init stub guard` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `4a0a5c4292f Tighten LCP init stub guard`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by nineteen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Math stub export-guard status:

- `scripts/check_lcp_solver_roster.py` now parses `__all__` lists through a
  shared AST helper and applies that guard to `python/stubs/dartpy/math.pyi` as
  well as `python/stubs/dartpy/__init__.pyi`.
- The roster check now rejects LCP solver classes that are defined in
  `math.pyi` but omitted from `math.pyi.__all__`, closing the remaining stub
  wildcard-export drift path.
- `python/tests/unit/test_check_lcp_solver_roster.py` now covers the missing
  `math.pyi.__all__` solver-class case alongside the existing math class,
  init import, and init `__all__` regressions.
- No generated stubs, bindings, solver predicates, demo runtime behavior,
  public APIs, checked profile CSVs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 14 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Init Stub Export Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0a48814439a Test LCP stub roster guard` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0a48814439a Test LCP stub roster guard`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by eighteen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Init stub export-guard status:

- `scripts/check_lcp_solver_roster.py` now parses
  `python/stubs/dartpy/__init__.pyi` with `ast` and checks LCP solver classes
  in both the `from .math import (...)` re-export list and `__all__`.
- This replaces the previous raw text search, which could pass if a solver name
  appeared in one init-stub surface while the actual re-export or wildcard
  export surface was stale.
- `python/tests/unit/test_check_lcp_solver_roster.py` now covers missing
  `.math` imports and missing `__all__` entries separately.
- No generated stubs, bindings, solver predicates, demo runtime behavior,
  public APIs, checked profile CSVs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 13 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Python Stub Guard Tests

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `e0431b507da Reuse LCP demo profile test schema` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `e0431b507da Reuse LCP demo profile test schema`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by seventeen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python stub guard-test status:

- `scripts/check_lcp_solver_roster.py` now keeps the Python stub exposure check
  in a focused `check_python_stub_solver_classes()` helper used by
  `check_roster()`.
- `python/tests/unit/test_check_lcp_solver_roster.py` now covers missing
  `python/stubs/dartpy/math.pyi` and `python/stubs/dartpy/__init__.pyi` solver
  classes directly, so future LCP solver additions do not silently lose stub
  coverage between the manifest, bindings, and dartpy stubs.
- No generated stubs, bindings, solver predicates, demo runtime behavior,
  public APIs, checked profile CSVs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 12 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Demo Test Schema Reuse

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `a948e32d42e Guard LCP demo profile schema` as the latest completed
   local tip before this checkpoint. If this section is committed, inspect
   `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `a948e32d42e Guard LCP demo profile schema`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by sixteen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Demo test schema-reuse status:

- `python/tests/unit/test_py_demo_panels.py` no longer carries a duplicate
  `_LCP_PROFILE_EVIDENCE_COLUMNS` tuple for LCP profile evidence fixtures.
- The panel-test evidence writer and missing/empty-column checks now reuse
  `lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS`, the same tuple
  validated by the demo runtime and guarded by the roster lint.
- No checked profile CSVs, benchmark registrations, solver predicates, demo
  runtime behavior, public APIs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_support_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_invalid_numeric_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_missing_evidence_columns python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_empty_evidence_file python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_missing_evidence_surfaces python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 27 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused panel tests, `scripts/check_lcp_solver_roster.py`, `pixi run lint`,
   and `git diff --check`, then commit the focused test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Demo Profile Schema Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `19ebc6db6cb Share LCP profile counter schema` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `19ebc6db6cb Share LCP profile counter schema`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fifteen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Demo profile schema-guard status:

- `scripts/check_lcp_solver_roster.py` now parses the Python LCP demo's
  display-side profile evidence schema constants, including category support
  fields, required surfaces, solver support fields, problem-type fields,
  identity schema version, family counters, and required columns.
- The roster lint gate now rejects future drift between the demo validator and
  the roster-owned profile evidence schema.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers both stale demo
  required-column tuples and stale non-column schema constants.
- No checked profile CSVs, benchmark registrations, solver predicates, demo
  runtime behavior, public APIs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 24 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused profile/roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile Counter Schema Reuse

Historical checkpoint section. It was the latest hand-off before the demo
profile schema-guard continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `e15d4321bf5 Reuse LCP profile evidence schema` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `e15d4321bf5 Reuse LCP profile evidence schema`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by fourteen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile counter schema-reuse status:

- `scripts/check_lcp_solver_roster.py` now owns the profile category, form
  support counter, problem-type counter, solver identity counter, and solver
  family counter constants used by profile evidence tooling.
- `scripts/lcp_performance_profile.py` imports those constants from the roster
  checker instead of carrying duplicate identity/family/support/problem-type
  definitions.
- `python/tests/unit/test_lcp_performance_profile.py` covers the profile
  generator's shared constants against the roster schema.
- No checked profile CSVs, benchmark registrations, solver predicates, demo
  runtime behavior, public APIs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 23 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused profile/roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile Generator Schema Reuse

Historical checkpoint section. It was the latest hand-off before the profile
counter schema-reuse continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `b79eb8892e7 Guard LCP profile native coverage` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `b79eb8892e7 Guard LCP profile native coverage`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by thirteen commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile generator schema-reuse status:

- `scripts/lcp_performance_profile.py` now reuses
  `scripts/check_lcp_solver_roster.py`'s `REQUIRED_EVIDENCE_COLUMNS` tuple when
  writing `performance_profile_evidence.csv`.
- The profile generator no longer carries a separate hardcoded evidence header
  that could drift from the roster checker and Python demo display schema.
- `python/tests/unit/test_lcp_performance_profile.py` covers the generated
  evidence CSV header against the shared roster schema.
- No checked profile CSVs, benchmark registrations, solver predicates, demo
  runtime behavior, public APIs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 22 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused profile/roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Profile Native Coverage Guard

Historical checkpoint section. It was the latest hand-off before the profile
generator schema-reuse continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `6695805b1c9 Guard LCP demo profile schema drift` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `6695805b1c9 Guard LCP demo profile schema drift`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by twelve commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile native coverage guard status:

- `scripts/check_lcp_solver_roster.py` now rejects checked performance-profile
  CSV headers that omit native solvers for the corresponding Standard, Boxed,
  or FrictionIndex profile surface.
- The same roster lint path now rejects checked
  `performance_profile_evidence.csv` files that omit all evidence rows for a
  native solver on a supported profile category.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers missing native
  solver regressions for both profile headers and profile evidence rows.
- No checked profile CSVs, benchmark registrations, solver predicates, demo
  runtime behavior, public APIs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 9 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Demo Profile Schema Drift Guard

Historical checkpoint section. It was the latest hand-off before the profile
native coverage guard continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `af41c467aad Validate LCP demo profile surfaces` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `af41c467aad Validate LCP demo profile surfaces`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by eleven commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Demo profile schema-drift guard status:

- `scripts/check_lcp_solver_roster.py` now parses the Python LCP demo's
  computed `_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS` tuple and compares
  it with the roster checker's `REQUIRED_EVIDENCE_COLUMNS`.
- The roster lint gate now rejects future drift where the checked evidence CSV
  schema and the Python demo's display-side evidence schema diverge.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers the new mismatch
  failure mode by simulating stale demo required columns.
- No checked profile CSVs, benchmark registrations, solver predicates, demo
  runtime behavior, public APIs, or performance timings were intentionally
  changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with 7 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused roster tests, `scripts/check_lcp_solver_roster.py`,
   `pixi run lint`, and `git diff --check`, then commit the focused
   script/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Python Demo Profile Surface Validation

Historical checkpoint section. It was the latest hand-off before the demo
profile schema-drift guard continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `0b8bbb74564 Validate LCP demo profile schema` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `0b8bbb74564 Validate LCP demo profile schema`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by ten commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile surface status:

- `python/examples/demos/scenes/lcp_physics.py` now requires the profile
  evidence summary source to include Standard, Boxed, and FrictionIndex rows
  before the Python LCP demo displays the evidence summary table.
- The demo rejects present-but-partial evidence files instead of silently
  omitting a profile surface from the table.
- `python/tests/unit/test_py_demo_panels.py` covers missing evidence surfaces
  alongside the existing schema, category, identity, support, problem-type, and
  numeric profile-evidence guards.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_support_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_invalid_numeric_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_missing_evidence_columns python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_empty_evidence_file python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_missing_evidence_surfaces python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 27 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-13 Current Continuation - Python Demo Profile Schema Validation

Historical checkpoint section. It was the latest hand-off before the Python
demo profile surface validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `e97bb97469d Validate LCP demo profile metrics` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `e97bb97469d Validate LCP demo profile metrics`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by nine commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile schema status:

- `python/examples/demos/scenes/lcp_physics.py` now validates the performance
  profile evidence CSV header before summarizing rows in the Python LCP demo
  panel.
- The demo rejects evidence files that are missing required columns and rejects
  header-only evidence files instead of presenting an empty summary.
- `python/tests/unit/test_py_demo_panels.py` covers missing required columns
  and empty evidence files alongside the existing category, identity, support,
  problem-type, and numeric profile-evidence guards.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_support_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_invalid_numeric_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_missing_evidence_columns python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_empty_evidence_file python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 26 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-12 Current Continuation - Python Demo Profile Numeric Validation

Historical checkpoint section. It was the latest hand-off before the Python
demo profile schema validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `761f4f967a6 Validate LCP demo profile support flags` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `761f4f967a6 Validate LCP demo profile support flags`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by eight commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile numeric-evidence status:

- `python/examples/demos/scenes/lcp_physics.py` now validates the numeric
  profile evidence fields consumed by the Python LCP demo summary before
  displaying them.
- The demo rejects invalid `problem_size`, `lcp_dimension`, `contact_count`,
  `time_ns`, `contract_ok`, `iterations`, `residual`, `complementarity`, and
  `bound_violation` rows, including FrictionIndex dimension/contact mismatches
  and non-finite metric values.
- `python/tests/unit/test_py_demo_panels.py` covers invalid numeric evidence
  rows while keeping the existing category, identity, support, and problem-type
  regressions in the same focused slice.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_support_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_invalid_numeric_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 24 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-12 Current Continuation - Python Demo Profile Support Validation

Historical checkpoint section. It was the latest hand-off before the Python
demo profile numeric validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `ee9ab77a1fb Validate LCP demo profile identities` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `ee9ab77a1fb Validate LCP demo profile identities`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by seven commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile support-counter status:

- `python/examples/demos/scenes/lcp_physics.py` now validates
  `solver_supports_standard`, `solver_supports_boxed`, and
  `solver_supports_friction_index` against the demo solver manifest before
  summarizing profile evidence rows.
- The demo still rejects non-native evidence rows and concrete
  `solver_supports_problem=0` native rows, but now also rejects stale all-form
  support counters on otherwise valid rows.
- `python/tests/unit/test_py_demo_panels.py` covers stale support counters for
  Standard `Dantzig` evidence rows and preserves separate coverage for
  non-native `Lemke` and unsupported `Dantzig` boxed rows.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_support_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 14 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-12 Current Continuation - Python Demo Profile Identity Validation

Historical checkpoint section. It was the latest hand-off before the Python
demo profile support-counter validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `52f22d39d30 Validate LCP demo profile problem types` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `52f22d39d30 Validate LCP demo profile problem types`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by six commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile identity status:

- `python/examples/demos/scenes/lcp_physics.py` now validates solver identity
  metadata before summarizing profile evidence rows in the Python LCP demo
  panel.
- The demo summary now rejects stale or mismatched
  `solver_identity_schema_version`, `solver_manifest_index`, solver-family
  one-hot counters, and unknown profile evidence solver names, keeping the
  display path aligned with the roster guard.
- `python/tests/unit/test_py_demo_panels.py` uses complete evidence-schema
  rows for profile-summary regressions and covers stale identity schema,
  manifest-index, family-counter, and unknown-solver rows.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 11 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-12 Current Continuation - Python Demo Profile Problem-Type Validation

Historical checkpoint section. It was the latest hand-off before the Python
demo profile identity validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `fdfcf322e26 Validate LCP demo profile evidence rows` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `fdfcf322e26 Validate LCP demo profile evidence rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by five commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile problem-type status:

- `python/examples/demos/scenes/lcp_physics.py` now validates profile evidence
  rows before summarizing them in the Python LCP demo panel.
- The demo summary now checks the full one-hot problem-type counter set,
  including `problem_type_invalid=0`, so unknown categories, category/type
  mismatches, non-native solver/category rows, and concrete
  `solver_supports_problem=0` rows fail before reaching the displayed summary.
- `python/tests/unit/test_py_demo_panels.py` covers non-native and unsupported
  boxed `Lemke` evidence rows plus unknown and mismatched problem-type rows.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 6 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-12 Current Continuation - Python Demo Profile Evidence Validation

Historical checkpoint section. It was the latest hand-off before the Python
demo profile problem-type validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `04cc867b958 Keep partial LCP profile checks strict` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `04cc867b958 Keep partial LCP profile checks strict`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by four commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Python demo profile-evidence status:

- `python/examples/demos/scenes/lcp_physics.py` now validates profile evidence
  rows before summarizing them in the Python LCP demo panel.
- The demo summary rejects unknown categories, rows whose solver is not native
  for the displayed surface, rows whose concrete `solver_supports_problem`
  counter is false, and rows whose problem-type counter does not match the
  category. This keeps the demo metadata path aligned with the stricter roster
  and profile guards.
- `python/tests/unit/test_py_demo_panels.py` covers non-native and unsupported
  boxed `Lemke` evidence rows.
- No checked profile CSVs, benchmark registrations, solver predicates, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target dartpy`
  passed after the existing built extension proved stale against the local
  shared libraries.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_profile_summary_rejects_non_native_evidence_rows python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  passed with 3 tests.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun the
   focused py-demo panel tests, `pixi run lint`, and `git diff --check`, then
   commit the focused demo/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.

## 2026-06-12 Current Continuation - Profile Evidence Partial-Run Validation

Historical checkpoint section. It was the latest hand-off before the first
Python demo profile-evidence validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `914e3cf4a47 Reject non-native LCP profile rows` as the latest
   completed local tip before this checkpoint. If this section is committed,
   inspect `git log --oneline --decorate -8` for the new exact tip.
3. Continue the broader LCP solver/interface/demo audit from one concrete gap
   at a time. Do not retire this dev-task folder yet.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state before this checkpoint commit:

- Branch: `feature/lcp-solver-interface-demos`.
- Current local tip before this edit:
  `914e3cf4a47 Reject non-native LCP profile rows`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`
  with the local branch ahead by three commits before this edit.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Profile-evidence native-row status:

- `scripts/check_lcp_solver_roster.py` now rejects performance-profile
  evidence rows whose solver/category pair is not native-supported. This closes
  the gap where an unsupported row could satisfy the manifest-derived
  `solver_supports_problem=0` expectation even though profile evidence is meant
  to contain native benchmark rows only.
- `scripts/lcp_performance_profile.py` now rejects non-native solver/category
  rows during coverage checks even when legacy benchmark JSON lacks the
  concrete `solver_supports_problem` counter.
- `scripts/lcp_performance_profile.py --allow-partial` now only relaxes
  missing native solver coverage for smoke runs. Invalid rows, unsupported
  `solver_supports_problem=0` rows, and schema/counter mismatches remain hard
  failures.
- `python/tests/unit/test_check_lcp_solver_roster.py` covers that regression
  with a boxed `Lemke` evidence row.
- `python/tests/unit/test_lcp_performance_profile.py` covers the corresponding
  boxed `Lemke` historical-row case plus strict `--allow-partial` behavior.
- No benchmark registrations, solver predicates, checked profile CSVs, public
  APIs, or performance timings were intentionally changed.

Verification completed in this continuation:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_lcp_performance_profile.py -q`
  passed with 18 tests.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed with 24 solvers, 23 standard, 15 boxed, and 16 findex.
- No cached `build/lcp_profile_full.json` exists in this checkout, so the
  cached profile-regeneration path was not rerun for this slice.
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on older
   handoff sections.
2. If this checkpoint is still uncommitted and files change again, rerun
   `pixi run lint` and `git diff --check`, then commit the focused
   checker/test/docs change.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.
4. Avoid a full performance-profile refresh while the host is under unrelated
   heavy build load unless the user explicitly asks for it.

## 2026-06-12 Current Continuation - SAP Matvec Reuse Verification

Historical checkpoint section. It was the latest hand-off before the
profile-evidence native-row validation continuation.

Fresh AI session priority:

1. Start from the current checkout, not from older WIP wording. Read
   `AGENTS.md`, `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Treat `700afe3d46b Checkpoint SAP matvec handoff WIP` as the current code
   checkpoint: the SAP matvec-reuse source edit is committed locally and on the
   tracking branch, not dirty worktree state.
3. Do not claim a SAP performance win from the checkpoint yet. Focused
   correctness and benchmark contract checks pass, but the timing comparison is
   inconclusive on the loaded host.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current tip:
  `700afe3d46b Checkpoint SAP matvec handoff WIP`.
- Current relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

SAP matvec-reuse status:

- `dart/math/lcp/other/sap_solver.cpp` reuses `ax = A * x` for the current
  quadratic cost and gradient, and reuses `axNew = A * xNew` for Armijo
  candidate cost evaluation.
- No algorithm policy, exact-path gate, tolerance, benchmark option, or public
  API was intentionally changed.
- The focused SAP tests and SAP benchmark contract slice now pass after the
  checkpoint.

Verification completed in this continuation:

- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`
  passed.
- `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers --gtest_filter='*Sap*'`
  passed with 3 tests:
  `BoxedProjectedActiveSetFastPath.SapUsesLinearSolve`,
  `SapSolverCoverage.DefaultCustomZeroRhsAndEdgeCases`, and
  `SapSolverCoverage.RegularizesIndefiniteHessian`.
- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target BM_LCP_COMPARE`
  passed.
- `build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='^BM_LcpCompare/(Boxed|FrictionIndex)/Sap/(12|24|48|4|16|64)$' --benchmark_format=json --benchmark_min_time=0.005s`
  passed; all six rows reported `contract_ok=1`, expected SAP solver-family
  metadata, and expected solver iteration counts (`0` for boxed exact-path
  rows, `2` for FrictionIndex rows).
- `pixi run lint` passed, including `lint-lcp-solver-roster` and
  `sync-ai-commands`.
- `git diff --check` passed.

Post-edit timing signal:

- The benchmark host was under load (`load_avg` about `17.7`, with an unrelated
  `task_1` CUDA build still running), so the timing comparison is not clean
  performance evidence.
- The sampled post-edit rows were about:
  - Boxed `Sap/12`: `1576.0 ns`, `iterations=0`, `contract_ok=1`.
  - Boxed `Sap/24`: `4142.9 ns`, `iterations=0`, `contract_ok=1`.
  - Boxed `Sap/48`: `15816.9 ns`, `iterations=0`, `contract_ok=1`.
  - FrictionIndex `Sap/4`: `1606.8 ns`, `iterations=2`,
    `contract_ok=1`.
  - FrictionIndex `Sap/16`: `15090.5 ns`, `iterations=2`,
    `contract_ok=1`.
  - FrictionIndex `Sap/64`: `318602.3 ns`, `iterations=2`,
    `contract_ok=1`.
- The boxed rows exercise SAP's exact fast path and are not expected to benefit
  from the iterative matvec reuse.

Immediate resume guidance:

1. Run `git status -sb` and inspect this top section before relying on the
   older WIP handoff below.
2. If files change again before a local commit, rerun `pixi run lint` and
   `git diff --check`.
3. Continue the broader LCP interface/demo audit from the next concrete gap.
   Do not treat the broad LCP objective as complete.
4. Do not retry the earlier rejected SAP FrictionIndex exact shortcut or
   ShockPropagation exact-path probe without a materially different hypothesis.

## 2026-06-12 Current Stop Handoff - SAP Matvec Reuse WIP

Historical checkpoint section. It was the latest hand-off before the SAP
matvec-reuse verification continuation.

Fresh AI session priority:

1. Start from the dirty worktree, not from memory. Read `AGENTS.md`,
   `docs/ai/principles.md`, this file, and `RESUME.md`.
2. Preserve the handoff before doing implementation. The user intent at this
   stop was safe continuation by a fresh AI session.
3. Treat the SAP matvec-reuse source edit as unverified WIP until focused
   post-edit tests and benchmarks prove it.
4. Do not push, open a PR, retry CI, or mutate GitHub state without explicit
   maintainer/user approval.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current tip before this stop handoff:
  `5e47facbfbe Extend boxed projection exact paths`.
- Latest `origin/main` refresh before this stop handoff:
  `bb851f45360 Add DART 7 architecture assessment, PLAN-091 hardening plan, and work-packet harness (#2986)`;
  merging `origin/main` into the branch reported `Already up to date`.
- Current branch relationship before this handoff:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Uncommitted worktree state at handoff:

- `dart/math/lcp/other/sap_solver.cpp` has an uncommitted WIP optimization.
  It adds `ax` and `axNew` scratch vectors in `SapSolver::solve(...)`, reuses
  `A * x` for both the current quadratic cost and gradient, and reuses an
  `A * xNew` scratch in the Armijo line-search cost.
- No algorithm policy, exact-path gate, tolerance, benchmark option, or public
  API was intentionally changed in that WIP source edit.
- `docs/dev_tasks/lcp_solver_interface_demos/README.md` and `RESUME.md` were
  updated for this stop handoff.

Evidence gathered before the stop:

- Baseline command before the source edit passed:
  `build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='^BM_LcpCompare/(Boxed|FrictionIndex)/Sap/(12|24|48|4|16|64)$' --benchmark_format=json --benchmark_min_time=0.005s`.
- Baseline sampled rows before the source edit:
  - Boxed `Sap/12`: about `1136.7 ns`, `iterations=0`, `contract_ok=1`.
  - Boxed `Sap/24`: about `3255.9 ns`, `iterations=0`, `contract_ok=1`.
  - Boxed `Sap/48`: about `13481.8 ns`, `iterations=0`, `contract_ok=1`.
  - FrictionIndex `Sap/4`: about `1412.4 ns`, `iterations=2`,
    `contract_ok=1`.
  - FrictionIndex `Sap/16`: about `11802.4 ns`, `iterations=2`,
    `contract_ok=1`.
  - FrictionIndex `Sap/64`: about `266911.0 ns`, `iterations=2`,
    `contract_ok=1`.

Verification status:

- No completed post-edit verification exists for the SAP WIP source edit.
- A combined build attempt failed because `scripts/cmake_build.py` accepts one
  `--target` per invocation:
  `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE`
  returned `unrecognized arguments: BM_LCP_COMPARE`.
- A follow-up build of
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` was interrupted by the
  user. The resulting `python scripts/cmake_build.py --target tests` and
  `ninja` processes were found still running and were stopped with `kill`.
- `pixi run lint`, `git diff --check`, focused C++ tests, and post-edit SAP
  benchmark comparisons have not been run after the WIP edit.

Immediate resume guidance:

1. Start with `git status -sb` and
   `git diff -- dart/math/lcp/other/sap_solver.cpp docs/dev_tasks/lcp_solver_interface_demos/README.md docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
2. Treat the SAP source edit as unverified WIP. Decide whether to keep and
   verify it or revert it before adding more solver changes.
3. If keeping it, rebuild the focused C++ target and benchmark target with one
   `scripts/cmake_build.py --target ...` invocation per target, then run the
   focused SAP tests/benchmark comparison, `pixi run lint`, and
   `git diff --check` before committing.
4. Do not retry the earlier rejected SAP FrictionIndex exact shortcut or
   ShockPropagation exact-path probe without a materially different hypothesis.

## 2026-06-13 Current Continuation - Boxed Projection Exact Paths

Historical checkpoint section. It was the latest hand-off before the SAP
matvec-reuse WIP stop handoff.

This was the latest hand-off state before the SAP matvec-reuse WIP stop
handoff. Sections below are historical checkpoints and may describe their own
local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current base before this checkpoint:
  `14e637307bc Refresh live LCP performance profiles`.
- Current branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 2]`.
- Checkpoint target: `Extend boxed projection exact paths`.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Implementation:

- `RedBlackGaussSeidelSolver` now routes non-warm-started, no-custom-option
  small boxed LCPs through `detail::trySolveProjectedActiveSetBoxedLcp(...)`
  before entering the two-color iteration.
- `SymmetricPsorSolver` now uses the same projected active-set boxed exact path
  under its existing profile-shaped exact-path size gate.
- New projection-solver unit tests assert that both solvers solve a small boxed
  active-set packet with `iterations == 0`.
- The stale Jacobi "large problem hits max iterations" test now explicitly uses
  the warm-started iterative path so it does not get bypassed by the existing
  Standard exact shortcut.
- The Python `lcp_physics` profile summary and panel assertions were refreshed
  after regenerating the live profile artifacts.

Current profile signal from the live run after this change:

- Standard: no solver average is above `1.6x`; `MPRGP` is about `1.48x`,
  followed by `RedBlackGaussSeidel` about `1.38x`.
- Boxed: `Sap` is the only current average above `1.6x` at about `1.68x`;
  `ShockPropagation` is about `1.54x`, `Admm` about `1.49x`, and
  `RedBlackGaussSeidel` / `SymmetricPsor` moved down to about `1.32x` /
  `1.38x`.
- FrictionIndex: `Sap` and `ShockPropagation` are the current above-`1.6x`
  rows at about `1.82x` and `1.73x`.

Verification completed so far:

- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target UNIT_math_lcp_math_lcp_lcp_projection_solvers`
  passed.
- `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_projection_solvers --gtest_filter='SymmetricPsorSolver.UsesProjectedActiveSetFastPathForSmallBoxedProblem:RedBlackGaussSeidelSolver.UsesProjectedActiveSetFastPathForSmallBoxedProblem:SymmetricPsorSolver.SolveBoxedProblem:RedBlackGaussSeidelSolver.LargeProblemHitsMaxIterations:RedBlackGaussSeidelSolver.ThreadedPathMatchesSerial'`
  passed with `5 tests`.
- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target BM_LCP_COMPARE`
  passed.
- `build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='^BM_LcpCompare/Boxed/(RedBlackGaussSeidel|SymmetricPsor)/(12|24|48)$' --benchmark_format=json --benchmark_min_time=0.001s`
  passed; all sampled rows reported `contract_ok=1` and `iterations=0`.
- `pixi run python scripts/lcp_performance_profile.py --run --cache build/lcp_profile_full.json --output docs/background/lcp/figures --benchmark-timeout 900`
  passed and regenerated all checked profile CSV artifacts.
- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target UNIT_math_lcp_math_lcp_lcp_projection_solvers && build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_projection_solvers`
  passed with `47 tests`.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `57 passed`.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is still uncommitted, run the focused C++/Python tests,
   `pixi run lint`, and `git diff --check`, then commit it with
   `Extend boxed projection exact paths`.
3. Continue from the next bounded performance/interface/demo gap. The strongest
   next performance target from this refresh is `Sap` on Boxed/FrictionIndex
   rows or FrictionIndex `ShockPropagation`.
4. Do not retry the earlier rejected SAP FrictionIndex exact shortcut or
   ShockPropagation exact-path probe without a materially different hypothesis.

## 2026-06-13 Current Continuation - Live Performance Profile Refresh

Historical checkpoint section. It was the latest hand-off before the boxed
projection exact-path continuation.

This was the latest hand-off state before the boxed projection exact-path
continuation. Sections below are historical checkpoints and may describe their
own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current base before this checkpoint:
  `d04616238f5 Expose LCP solver family evidence counters`.
- Current branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 1]`.
- Checkpoint target: `Refresh live LCP performance profiles`.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

Benchmark refresh:

- Ran the live performance-profile refresh instead of reusing the cached JSON:
  `pixi run python scripts/lcp_performance_profile.py --run --cache build/lcp_profile_full.json --output docs/background/lcp/figures --benchmark-timeout 900`.
- The command regenerated `build/lcp_profile_full.json` and refreshed the
  checked profile artifacts:
  `performance_profile_evidence.csv`, `performance_profile_standard.csv`,
  `performance_profile_boxed.csv`, and `performance_profile_frictionindex.csv`.
- The Python `lcp_physics` performance-profile metadata and panel assertions
  were updated to match the refreshed profile rather than the older cached
  profile.

Current profile signal from the live run:

- Standard: no solver average is above `1.6x`; `MPRGP` is the largest current
  row at about `1.48x`, followed by `Pgs`, `RedBlackGaussSeidel`, `Lemke`,
  `Baraff`, `PenalizedFischerBurmeisterNewton`, `Tgs`, and `NNCG`.
- Boxed: no solver average is above `1.6x`; `RedBlackGaussSeidel` is about
  `1.58x`, `SymmetricPsor` about `1.54x`, and `ShockPropagation` about
  `1.47x`.
- FrictionIndex: no solver average is above `1.6x`; `ShockPropagation` is
  about `1.55x`, `Sap` about `1.48x`, and `Admm` about `1.46x`.

Verification completed so far:

- Live profile refresh command above passed and saved all four CSV artifacts.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `57 passed`.
- `pixi run lint` passed, including `lint-lcp-solver-roster`.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is still uncommitted, run the focused profile/roster/py-demo
   tests, `pixi run lint`, and `git diff --check`, then commit it with
   `Refresh live LCP performance profiles`.
3. Continue from the next bounded performance/interface/demo gap. The strongest
   next performance target from this refresh is Boxed `RedBlackGaussSeidel` /
   `SymmetricPsor`, or FrictionIndex `ShockPropagation` / `Sap` / `Admm`.
4. Do not retry the rejected SAP FrictionIndex exact shortcut or
   ShockPropagation exact-path probe without a materially different hypothesis.

## 2026-06-13 Current Continuation - Solver Family Evidence Counters

Historical checkpoint section. It was the latest hand-off before the live
performance-profile refresh.

This was the latest hand-off state before the live refresh. Sections below are
historical checkpoints and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current base before this checkpoint:
  `dcfe8aa0ea0 Update LCP handoff after stop request`.
- Current branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos`.
- Checkpoint target: `Expose LCP solver family evidence counters`.
- Checkpoint status: verify with `git log --oneline --decorate -3`; if the
  `Expose LCP solver family evidence counters` commit is absent, the slice was
  interrupted before checkpointing.
- This branch has no associated PR. Do not push, open a PR, or mutate GitHub
  state without explicit maintainer/user approval.

DART 7 harness alignment:

- `BM_LcpCompare` now emits one-hot solver-family counters:
  `solver_family_pivoting`, `solver_family_projection`,
  `solver_family_newton`, and `solver_family_other`.
- `scripts/lcp_performance_profile.py` reads those counters when present and
  derives the same counters from the C++ solver manifest for cached rows, so
  the checked evidence CSV can be refreshed without rerunning the full
  benchmark suite.
- `scripts/check_lcp_solver_roster.py` now requires those family counters in
  the checked evidence CSV and validates that they are one-hot and match the
  manifest family for each solver.
- The Python `lcp_physics` panel's evidence schema table now documents the
  solver-family counters next to the manifest-index and support counters.
- `docs/background/lcp/figures/performance_profile_evidence.csv` was refreshed
  from `build/lcp_profile_full.json` with the added columns. The expensive
  benchmark suite was not rerun in this slice.

Verification completed so far:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `57 passed`.
- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target BM_LCP_COMPARE`
  passed.
- `build/default/cpp/Release/bin/BM_LCP_COMPARE --benchmark_filter='^BM_LcpCompare/Standard/Dantzig/12$' --benchmark_format=json --benchmark_min_time=0.001s`
  passed and emitted `solver_family_pivoting=1` with the other family counters
  at `0` for the Dantzig row.
- `pixi run lint` passed, including `lint-lcp-solver-roster`.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If the `Expose LCP solver family evidence counters` commit is missing, rerun
   the focused tests, `pixi run lint`, and `git diff --check`, then commit it.
3. Continue from the next bounded DART 7 LCP interface/demo/performance gap.
   The broad LCP objective is still incomplete.
4. Do not retry the rejected SAP FrictionIndex exact shortcut or
   ShockPropagation exact-path probe without a materially different hypothesis.

## 2026-06-12 Current Stop Handoff - Consolidated Branch

Historical checkpoint section. It was the latest hand-off before the
solver-family evidence-counter continuation.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Latest `origin/main` refresh:
  - `git fetch origin main` failed because SSH to `github.com:22` is not
    reachable in this environment.
  - `git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main`
    succeeded.
  - `git merge --no-edit origin/main` reported `Already up to date`.
- Current branch relationship before this docs-only hand-off checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 90]`.
- Current tip before this docs-only hand-off checkpoint:
  `8b571a91031 Expose LCP problem contact metadata`.
- Local branch cleanup check found only two local branches:
  `feature/lcp-solver-interface-demos` and `main`. No extra local task branches
  created by this session remain to delete.
- The current branch is the consolidated resume branch. Do not create another
  task branch unless a future maintainer explicitly wants a split.
- The user requested no further implementation work; keep any remaining edits
  to hand-off docs and repository hygiene only.
- This branch is approved by the user for push after the hand-off docs are
  updated.

Latest completed implementation checkpoint:

- `8b571a91031 Expose LCP problem contact metadata` is committed locally.
- That checkpoint exposed `LcpProblem` FrictionIndex row/contact metadata in
  C++, dartpy, the Python representative solver suite, and the LCP benchmark
  counter path.
- Verification for that checkpoint was already completed before this stop
  hand-off: focused C++ LCP type test, dartpy rebuild, focused Python LCP/demo
  tests, LCP benchmark target compile, `pixi run lint`, and
  `git diff --check`.

Immediate resume guidance:

1. Start with `git checkout feature/lcp-solver-interface-demos`.
2. Run `git status -sb` and `git log --oneline --decorate -8`.
3. Continue only if the user explicitly resumes implementation. The latest
   bounded next area inspected, but not implemented, was DART 7 harness-aligned
   LCP solver identity/support metadata in benchmark/demo evidence.
4. Do not retry the rejected SAP FrictionIndex exact shortcut or
   ShockPropagation exact-path probe without a materially different hypothesis.

## 2026-06-12 Current Continuation - LcpProblem Contact Metadata

Historical checkpoint section. It was the latest hand-off before
`8b571a91031` was committed.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable:
  `git fetch https://github.com/dartsim/dart.git main`.
- `git merge --no-edit FETCH_HEAD` reported `Already up to date`, so the
  branch remains current with the PR #2986 DART 7 work-packet harness on
  `main` (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 89]`.
- Last committed checkpoint:
  `71bfc925863 Expose LCP representative requirement coverage`.
- Checkpoint target:
  `Expose LCP problem contact metadata`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 90 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- `dart::math::LcpProblem` now exposes
  `getFrictionIndexRowCount()` and `getFrictionIndexContactCount()` so
  DART 7 LCP evidence can derive contact-structure metadata from the problem
  representation itself rather than repeating per-benchmark assumptions.
- dartpy exposes matching
  `LcpProblem.get_friction_index_row_count()` and
  `LcpProblem.get_friction_index_contact_count()` helpers.
- The Python `lcp_physics` representative solver suite now records and renders
  LCP row dimension plus FrictionIndex contact count in its summary table.
- The LCP comparison benchmark path now fills `contact_count` centrally for
  FrictionIndex problems solved through the shared benchmark runner.
- `python/tests/unit/math/test_lcp.py` now accepts any valid solution for a
  degenerate indefinite Baraff fallback case, because that problem has multiple
  valid LCP solutions and the previous exact-vector assertion was brittle.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/lcp_types.hpp`
- `python/dartpy/math/lcp.cpp`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/math/test_lcp.py`
- `python/tests/unit/test_py_demo_panels.py`
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`
- `tests/unit/math/lcp/test_lcp_types.cpp`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `pixi run run-cpp-target UNIT_math_lcp_math_lcp_lcp_types` passed with
  `15 tests` passed.
- `pixi run build-py-dev` passed and rebuilt dartpy with the new bindings.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/math/test_lcp.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `122 passed`.
- `CMAKE_BUILD_DIR=build/default/cpp/Release pixi run python scripts/cmake_build.py --target BM_LCPSOLVER`
  passed, compiling the touched LCP benchmark target without running the full
  benchmark.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is still uncommitted and no files changed after the
   verification above, commit it with `Expose LCP problem contact metadata`.
   If files changed, rerun `pixi run lint` and `git diff --check` first.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Representative Requirement Coverage

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`
  (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 88]`.
- Last committed checkpoint:
  `aa43914b348 Expose LCP solver selection guidance`.
- Checkpoint target:
  `Expose LCP representative requirement coverage`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 89 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Py-demo representative-example alignment:

- The Python `lcp_physics` panel now includes a Representative requirement
  coverage table that ties named evaluation needs to live packets, benchmark
  packets, metrics, and evidence cues.
- The table explicitly covers billiards symmetry/energy/momentum, high
  mass-ratio stacks, thin card piles, scalability smoke, and friction coupling
  with active tangential bounds.
- The setup metadata exposes `representative_requirement_rows`, and the panel
  test asserts these rows so future demo edits do not hide the requirement map.

Current dirty files before commit:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with `43 passed`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Expose LCP representative requirement coverage`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Solver Selection Guidance Panel

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`
  (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 87]`.
- Last committed checkpoint:
  `5d9ac41a227 Expose LCP profile evidence coverage summary`.
- Checkpoint target:
  `Expose LCP solver selection guidance`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 88 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Py-demo representative-example alignment:

- The Python `lcp_physics` panel now includes a Solver selection guide table
  that maps solver families to best-fit representative problem types,
  strengths, tradeoffs, and current profile-evidence cues.
- The table covers pivoting/direct, projection iterations, block/contact
  structure, Newton/interior/QP, and accelerated/splitting families.
- The setup metadata exposes `solver_guidance_rows`, and the panel test asserts
  the guidance is present so future demo edits keep solver pros/cons visible.

Current dirty files before commit:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with `43 passed`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Expose LCP solver selection guidance`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Profile Evidence Coverage Summary

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`
  (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 86]`.
- Last committed checkpoint:
  `128a054ff1a Expose LCP profile evidence schema`.
- Checkpoint target:
  `Expose LCP profile evidence coverage summary`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 87 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Py-demo DART 7 harness alignment:

- The Python `lcp_physics` panel now reads the checked
  `docs/background/lcp/figures/performance_profile_evidence.csv` artifact and
  shows a `lcp_performance_profile_evidence_summary` table under Performance
  profiles.
- The summary makes the current evidence coverage visible per profile surface:
  row count, solver count, LCP dimensions, FrictionIndex contact counts,
  `contract_ok` pass counts, max iterations, and max residual,
  complementarity, and bound violation.
- The setup metadata exposes `performance_profile_evidence_summary_rows`, so a
  fresh session can confirm the checked evidence CSV covers Standard, Boxed,
  and FrictionIndex rows without running the full benchmark refresh.

Current dirty files before commit:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with `43 passed`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Expose LCP profile evidence coverage summary`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Profile Evidence Schema Panel

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`
  (`bb851f453606`).
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 85]`.
- Last committed checkpoint:
  `7298ccde11c Expose LCP representative challenge labels`.
- Checkpoint target:
  `Expose LCP profile evidence schema`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 86 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Py-demo DART 7 harness alignment:

- The Python `lcp_physics` panel now includes a
  `lcp_performance_profile_evidence_schema` table under Performance profiles.
- The table makes the checked evidence CSV's harness-critical counters visible
  in the demo: solver identity schema/version, solver manifest index, native
  support flags, problem-type counters, `lcp_dimension`, `contact_count`,
  `contract_ok`, `iterations`, `residual`, `complementarity`, and
  `bound_violation`.
- The setup metadata also exposes
  `performance_profile_evidence_schema_rows` so headless tests and future
  hand-off sessions can assert the evidence contract without parsing the CSV.

Current dirty files before commit:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with `43 passed`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Expose LCP profile evidence schema`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Representative Challenge Labels

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS earlier in this continuation because
  SSH to `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 84]`.
- Last committed checkpoint:
  `59595b8d2f1 Record LCP problem dimension evidence`.
- Checkpoint target:
  `Expose LCP representative challenge labels`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 85 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Py-demo representative-example alignment:

- The Python `lcp_physics` panel's Representative solver suite now displays
  each standalone problem's `Challenge` text instead of hiding it only in the
  setup metadata.
- The visible suite now calls out the all-solver case intent directly:
  large-mass-ratio conditioning, active bounds, rank-deficient degeneracy,
  coupled friction-index active tangential bounds, and scalability smoke.
- This complements the existing live world packets for billiards
  symmetry/momentum/energy, high-mass-ratio stack drift, and thin card-pile
  spread/height loss.

Current dirty files before commit:

- `CHANGELOG.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q`
  passed with `43 passed`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Expose LCP representative challenge labels`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Problem Dimension Evidence Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 83]`.
- Last committed checkpoint:
  `de016b4969f Record LCP profile metric evidence`.
- Checkpoint target:
  `Record LCP problem dimension evidence`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 84 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- This continues the PR #2986 solver-family-intake rule that benchmark evidence
  must machine-record problem identity and metrics, not just rely on benchmark
  name parsing.
- `scripts/lcp_performance_profile.py` now preserves the emitted
  `problem_size` counter as `lcp_dimension` and the emitted `contact_count`
  counter for friction-index rows.
- Profile `problem_size` remains the profile argument. For Standard/Boxed rows
  it equals the LCP row dimension; for FrictionIndex rows it is contact count
  and the emitted `lcp_dimension` is `3 * contact_count`.
- Both profile ingestion and `scripts/check_lcp_solver_roster.py` reject stale
  rows where Standard/Boxed dimensions disagree, or where FrictionIndex
  `contact_count`/`lcp_dimension` do not match the profile size.
- The checked evidence artifact was regenerated from the existing
  `build/lcp_profile_full.json` cache, not from a new benchmark run.

Current dirty files before commit:

- `CHANGELOG.md`
- `docs/background/lcp/figures/performance_profile_evidence.csv`
- `scripts/lcp_performance_profile.py`
- `scripts/check_lcp_solver_roster.py`
- `python/tests/unit/test_lcp_performance_profile.py`
- `python/tests/unit/test_check_lcp_solver_roster.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --cache build/lcp_profile_full.json --output docs/background/lcp/figures`
  completed and regenerated the evidence CSV with populated dimension/contact
  columns.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with `12 passed`.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed.
- `pixi run lint` passed, including `lint-lcp-solver-roster` with the expanded
  dimension/contact evidence guard.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Record LCP problem dimension evidence`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Profile Metric Evidence Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the branch is current with the PR #2986
  DART 7 work-packet harness on `origin/main`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 82]`.
- Last committed checkpoint:
  `8aaa8e4548b Validate LCP performance evidence roster identity`.
- Checkpoint target:
  `Record LCP profile metric evidence`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 83 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- PR #2986 added the packet harness and tightened
  `docs/plans/solver-family-intake.md`; item 10 requires machine-recorded
  solver identity and metrics evidence for benchmark/evidence packets.
- `scripts/lcp_performance_profile.py` now preserves `iterations` and
  `bound_violation` from current `BM_LcpCompare` JSON rows and writes them to
  `docs/background/lcp/figures/performance_profile_evidence.csv`.
- `scripts/check_lcp_solver_roster.py` now requires the evidence CSV to carry
  finite positive `time_ns`, `contract_ok=1`, nonnegative integer
  `iterations`, and finite nonnegative residual/complementarity/bound-violation
  metrics, in addition to the identity/support/problem-type guards from the
  previous checkpoint.
- The checked evidence artifact was regenerated from the existing
  `build/lcp_profile_full.json` cache, not from a new benchmark run.

Current dirty files before commit:

- `CHANGELOG.md`
- `docs/background/lcp/figures/performance_profile_evidence.csv`
- `scripts/lcp_performance_profile.py`
- `scripts/check_lcp_solver_roster.py`
- `python/tests/unit/test_lcp_performance_profile.py`
- `python/tests/unit/test_check_lcp_solver_roster.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main`
  and `git merge --no-edit origin/main`: up to date.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --cache build/lcp_profile_full.json --output docs/background/lcp/figures`
  completed and regenerated the evidence CSV with populated metric columns.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with `10 passed`.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed.
- `pixi run lint` passed, including `lint-lcp-solver-roster` with the expanded
  evidence metric guard.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Record LCP profile metric evidence`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Evidence CSV Roster Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation; merging
  `origin/main` reported `Already up to date`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 81]`.
- Last committed checkpoint:
  `27bd1f8fe8d Record LCP benchmark solver identity counters`.
- Checkpoint target:
  `Validate LCP performance evidence roster identity`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 82 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- `scripts/check_lcp_solver_roster.py` now validates
  `docs/background/lcp/figures/performance_profile_evidence.csv`, not just the
  profile CSV headers.
- The guard checks each evidence row's category, solver, positive problem
  size, `solver_identity_schema_version`, `solver_manifest_index`, native
  support counters, concrete `solver_supports_problem`, and one-hot
  `problem_type_*` counters against the C++ LCP solver manifest.
- This makes the checked row-level evidence artifact part of `pixi run lint`,
  so stale identity-less or mismatched profile evidence fails locally.

Current dirty files before commit:

- `CHANGELOG.md`
- `scripts/check_lcp_solver_roster.py`
- `python/tests/unit/test_check_lcp_solver_roster.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py -q`
  passed with `2 passed`.
- `PYTHONPATH=python pixi run python scripts/check_lcp_solver_roster.py`
  passed.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_check_lcp_solver_roster.py python/tests/unit/test_lcp_performance_profile.py -q`
  passed with `9 passed`.
- `pixi run lint` passed, including `lint-lcp-solver-roster` with the new
  evidence CSV guard.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with
   `Validate LCP performance evidence roster identity`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Solver Identity Counters

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS in this continuation because SSH to
  `github.com:22` was not reachable. `git merge --no-edit origin/main`
  reported `Already up to date`, so the PR #2986 DART 7 harness remains present
  via merge commit `bb851f45360`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 80]`.
- Last committed checkpoint:
  `f27d12c110d Add LCP performance profile evidence CSV`.
- Checkpoint target:
  `Record LCP benchmark solver identity counters`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 81 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- `tests/common/lcpsolver/lcp_solver_manifest.hpp` now owns
  `kLcpSolverIdentitySchemaVersion = 1` and a stable 1-based manifest index
  helper for benchmark identity evidence.
- `BM_LcpCompare` rows now emit `solver_identity_schema_version` and
  `solver_manifest_index` counters in addition to the previous support and
  problem-type counters.
- `scripts/lcp_performance_profile.py` preserves those identity counters,
  rejects current-schema rows whose benchmark-name solver disagrees with the
  manifest index, and writes both fields into
  `docs/background/lcp/figures/performance_profile_evidence.csv`.
- The Python LCP demo performance-profile metadata was refreshed from the new
  full-profile run and continues to point at the row-level evidence CSV.

Current dirty files before commit:

- `CHANGELOG.md`
- `tests/common/lcpsolver/lcp_solver_manifest.hpp`
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`
- `scripts/lcp_performance_profile.py`
- `docs/background/lcp/figures/performance_profile_evidence.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_lcp_performance_profile.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main`
  and `git merge --no-edit origin/main`: up to date.
- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with `7 passed`.
- `cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"`
  passed with the safe parallelism cap.
- `pixi run bm lcp_compare -- --benchmark_filter='BM_LcpCompare/Standard/Dantzig/12$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_out=build/lcp_identity_counters_probe.json --benchmark_out_format=json`
  passed; the JSON row emitted `solver_identity_schema_version=1.0` and
  `solver_manifest_index=1.0`.
- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --run --cache build/lcp_profile_full.json --output docs/background/lcp/figures --benchmark-timeout 900`
  completed, cached current-schema benchmark JSON, and regenerated the checked
  profile/evidence CSVs with populated identity counters.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `50 passed`.
- `pixi run lint` passed; it reformatted
  `scripts/lcp_performance_profile.py`.
- After lint, the focused parser/demo tests were rerun and passed with
  `50 passed`.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with
   `Record LCP benchmark solver identity counters`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; do not retry the
   rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Row-Level Profile Evidence

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed earlier in this continuation; merging
  `origin/main` reported "Already up to date." The recent DART 7 harness from
  PR #2986 is already in this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 79]`.
- Last committed checkpoint:
  `a966dc95c9e Validate LCP benchmark form support evidence`.
- Checkpoint target:
  `Add LCP performance profile evidence CSV`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 80 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- `scripts/lcp_performance_profile.py` now writes
  `performance_profile_evidence.csv` beside the checked profile CSVs. The
  evidence file records each parsed profile row's category, manifest solver,
  problem size, timing, contract metrics, emitted form-support counters,
  concrete `solver_supports_problem`, and emitted `problem_type_*` counters.
- `python/examples/demos/scenes/lcp_physics.py` now exposes that evidence CSV
  in each performance-profile metadata row and GUI table, so the demo points to
  the row-level support evidence behind the apples-to-apples curves.
- The checked Standard, Boxed, and FrictionIndex performance-profile CSVs and
  the new evidence CSV were regenerated from current-schema `BM_LcpCompare/`
  benchmark JSON, not the older historical cache that lacked support counters.

Current dirty files before commit:

- `CHANGELOG.md`
- `scripts/lcp_performance_profile.py`
- `docs/background/lcp/figures/performance_profile_evidence.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_lcp_performance_profile.py`
- `python/tests/unit/test_py_demo_panels.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=python pixi run python scripts/lcp_performance_profile.py --run --cache build/lcp_profile_full.json --output docs/background/lcp/figures --benchmark-timeout 900`
  completed, cached current-schema benchmark JSON, and wrote all profile CSVs.
- The regenerated evidence CSV has 184 lines including the header, with support
  and problem-type counters populated.
- `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py python/tests/unit/test_py_demo_panels.py -q`
  passed with `49 passed`.
- `pixi run lint` passed and reformatted
  `scripts/lcp_performance_profile.py`; the focused parser/demo tests were
  rerun afterwards and still passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Add LCP performance profile evidence CSV`.
3. Continue from a new bounded DART 7 LCP interface/demo gap; avoid retrying
   the rejected SAP FrictionIndex exact shortcut or ShockPropagation exact-path
   probe without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Form Support Schema Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed earlier in this continuation; merging
  `origin/main` reported "Already up to date." The recent DART 7 harness from
  PR #2986 is already in this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 78]`.
- Last committed checkpoint:
  `3cd4d9382c9 Validate LCP benchmark problem type evidence`.
- Checkpoint target:
  `Validate LCP benchmark form support evidence`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 79 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- `scripts/lcp_performance_profile.py` now retains the emitted form-level
  `solver_supports_standard`, `solver_supports_boxed`, and
  `solver_supports_friction_index` counters.
- Current-schema profile ingestion rejects rows where the
  `BM_LcpCompare/<problem-family>/...` category disagrees with the solver's
  emitted form-level native support counter.
- Historical cached rows without `solver_supports_*` form counters remain
  accepted for inspection.
- Python unit coverage now checks unsupported concrete solver/problem rows,
  form-support mismatch rows, problem-type/name mismatch rows, and historical
  row compatibility.

Current dirty files before commit:

- `CHANGELOG.md`
- `scripts/lcp_performance_profile.py`
- `python/tests/unit/test_lcp_performance_profile.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with `5 passed`.
- Focused parser acceptance check against `build/lcp_support_counters_probe.json`
  from the earlier benchmark probe passed.
- `pixi run lint` passed and reformatted
  `scripts/lcp_performance_profile.py`; the focused parser tests were rerun
  afterwards and still passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Validate LCP benchmark form support evidence`.
3. Continue from a new bounded DART 7 harness gap; avoid retrying the rejected
   SAP FrictionIndex exact shortcut or ShockPropagation exact-path probe
   without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Problem Type Schema Guard

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed earlier in this continuation; merging
  `origin/main` reported "Already up to date." The recent DART 7 harness from
  PR #2986 is already in this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 77]`.
- Last committed checkpoint:
  `99b68ab7ca0 Record concrete LCP benchmark support evidence`.
- Checkpoint target:
  `Validate LCP benchmark problem type evidence`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 78 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- `scripts/lcp_performance_profile.py` now retains all emitted
  `problem_type_*` counters and rejects current-schema rows where the
  `BM_LcpCompare/<problem-family>/...` name disagrees with the concrete problem
  type counters.
- The existing historical-cache compatibility path remains open: cached rows
  without `problem_type_*` counters are still accepted for inspection.
- Python unit coverage now checks parser retention, unsupported concrete
  solver/problem rejection, historical-row compatibility, and problem-type/name
  mismatch rejection.

Current dirty files before commit:

- `CHANGELOG.md`
- `scripts/lcp_performance_profile.py`
- `python/tests/unit/test_lcp_performance_profile.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with `4 passed`.
- Focused parser acceptance check against `build/lcp_support_counters_probe.json`
  from the previous benchmark probe passed; the current-schema Dantzig row is
  accepted with matching problem-type counters.
- `pixi run lint` passed and reformatted
  `scripts/lcp_performance_profile.py`; the focused parser tests were rerun
  afterwards and still passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Validate LCP benchmark problem type evidence`.
3. Continue from a new bounded DART 7 harness gap; avoid retrying the rejected
   SAP FrictionIndex exact shortcut or ShockPropagation exact-path probe
   without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Concrete Benchmark Support Evidence

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- `origin/main` was refreshed over HTTPS because SSH to GitHub port 22 was not
  reachable in this environment; `git merge --no-edit origin/main` reported
  "Already up to date." The recent DART 7 harness from PR #2986 is already in
  this branch via the earlier `origin/main` merge.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 76]`.
- Last committed checkpoint:
  `58dec3fd07a Record LCP post-APGD probe handoff`.
- Checkpoint target:
  `Record concrete LCP benchmark support evidence`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 77 commits.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- The `BM_LcpCompare` benchmark rows now machine-record concrete support
  evidence, not just benchmark-name identity:
  `solver_supports_standard`, `solver_supports_boxed`,
  `solver_supports_friction_index`, `solver_supports_problem`, and one-hot
  `problem_type_*` counters.
- `scripts/lcp_performance_profile.py` preserves `solver_supports_problem` and
  rejects current-schema profile rows that report unsupported concrete
  solver/problem pairs. Historical cached benchmark JSON without this counter
  remains accepted so older evidence packets can still be inspected.
- The new Python unit coverage guards both the strict current-schema rejection
  and the historical-cache compatibility path.

Current dirty files before commit:

- `CHANGELOG.md`
- `scripts/lcp_performance_profile.py`
- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`
- `python/tests/unit/test_lcp_performance_profile.py`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Verification completed for this checkpoint:

- `PYTHONPATH=python pixi run python -m pytest python/tests/unit/test_lcp_performance_profile.py -q`
  passed with `3 passed`.
- `cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"`
  rebuilt the benchmark target.
- `pixi run bm lcp_compare -- --benchmark_filter='BM_LcpCompare/Standard/Dantzig/12$' --benchmark_min_time=0.001s --benchmark_repetitions=1 --benchmark_out=build/lcp_support_counters_probe.json --benchmark_out_format=json`
  emitted the new support counters; the parsed row reported
  `solver_supports_problem=1.0` and `problem_type_standard=1.0`.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this slice is uncommitted, review the verification above and commit it
   with `Record concrete LCP benchmark support evidence`.
3. Continue from a new bounded DART 7 harness gap; avoid retrying the rejected
   SAP FrictionIndex exact shortcut or ShockPropagation exact-path probe
   without a materially different hypothesis.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Post-APGD Probe Triage

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 75]`.
- Last committed checkpoint:
  `c6eb366c126 Raise APGD exact gate for friction-index comparison`.
- Current worktree state before this docs checkpoint: clean.
- This branch has not been pushed in this continuation. No PR is associated
  with this branch yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- The accepted APGD gate slice remains the latest code checkpoint. It follows
  the PR #2986 packet-harness stance: evidence uses `BM_LcpCompare` benchmark
  names for problem family, manifest solver name, and size, plus `contract_ok`,
  timing fields, manifest metadata, and native support checks.
- After that checkpoint, two follow-up probes were evaluated and intentionally
  not kept because the focused packet evidence did not support source changes.

Rejected or non-actionable probes after `c6eb366c126`:

- Rejected SAP FrictionIndex exact shortcut:
  - Baseline file: `build/findex_sap_exact_baseline.json`.
  - Probe file: `build/findex_sap_exact_probe.json`.
  - Temporary source edit added
    `detail::trySolveInteriorFrictionIndexLcp(...)` to `SapSolver` behind a
    192-row gate, then was reverted.
  - Focused timings were mixed despite moving counters to `iterations=0`:
    `Sap/4` `1376ns -> 1101ns`, `Sap/16` `11312ns -> 11402ns`,
    `Sap/64` `224838ns -> 234951ns`.
  - Do not retry this exact shortcut unless there is a materially different
    hypothesis.
- Non-actionable ShockPropagation focus check:
  - Focus file: `build/shock_current_focus.json`.
  - Boxed and FrictionIndex `ShockPropagation` comparison rows already report
    `iterations=0` and `contract_ok=1`.
  - Representative current timings:
    Boxed `12/24/48` are `1170ns`, `3074ns`, `11466ns`; FrictionIndex
    `4/16/64` are `1142ns`, `11641ns`, `223086ns`.
  - The remaining `ShockPropagation` profile ratio is therefore not an obvious
    missing exact-path bug.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -8`.
2. If this docs checkpoint is uncommitted, run `pixi run lint`, run
   `git diff --check`, and commit the docs with a hand-off message.
3. For further implementation, prefer a new bounded hypothesis rather than
   retrying rejected Boxed `SymmetricPsor`/`RedBlackGaussSeidel` exact probes,
   rejected SAP FrictionIndex exact shortcut, or ShockPropagation exact-path
   investigation.
4. Current regenerated profiles have no Standard, Boxed, or FrictionIndex
   solver average above `1.6x`; any next performance slice should justify why
   it still needs source churn under the packet harness.
5. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - APGD FrictionIndex Gate

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 74]`.
- Last committed checkpoint:
  `63bfc8b349b Raise Jacobi standard exact gate and use LLT`.
- Checkpoint target:
  `Raise APGD exact gate for friction-index comparison`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 75 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- This is a bounded packet-like slice under the PR #2986 harness constraints:
  the accepted code change is the `ApgdSolver` exact shortcut gate used by the
  64-contact FrictionIndex comparison packet.
- The evidence packet uses the current interim LCP identity path: benchmark
  names encode problem family, manifest solver name, and size; the profile
  script derives rows from those names plus `contract_ok` and timing fields;
  the Python demo exposes manifest metadata and native support checks.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/projection/apgd_solver.cpp`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Current implementation slice:

- `ApgdSolver` now allows validated exact shortcuts through 192 scalar rows so
  the 64-contact FrictionIndex comparison packet can take the exact shortcut.
- The Standard 12/24/48 rows were already exact; their focused movement in this
  slice is treated as benchmark noise. The Standard 96 row also becomes exact
  under the raised gate.

Focused and profile evidence:

- Baseline:
  `build/findex_apgd_gate_baseline.json`.
- Prior Standard focused reference:
  `build/standard_sap_apgd_llt_probe.json`.
- Accepted focused probe:
  `build/apgd_gate192_probe.json`.
- Focused FrictionIndex `Apgd` timings moved approximately:
  - `Apgd/4`: `1119.00ns -> 1123.00ns`.
  - `Apgd/16`: `12956.00ns -> 11174.00ns`.
  - `Apgd/64`: `358923.00ns -> 231923.00ns`.
- Focused Standard `Apgd/96` moved approximately:
  `58359.00ns -> 41387.00ns`; smaller Standard APGD rows were already exact.
- Latest regenerated profile highlights:
  - Standard: `Apgd 1.22`; no solver average is above `1.6x`; highest rows are
    `BoxedSemiSmoothNewton 1.51`, `MPRGP 1.41`, and
    `RedBlackGaussSeidel 1.34`.
  - Boxed: no solver average is above `1.6x`; highest rows are
    `SymmetricPsor 1.57`, `RedBlackGaussSeidel 1.53`,
    `ShockPropagation 1.52`, and `SubspaceMinimization 1.41`.
  - FrictionIndex: no solver average is above `1.6x`; `Apgd` moved to `1.38`;
    highest rows are `Sap 1.58`, `ShockPropagation 1.58`, `BGS 1.45`, and
    `Admm 1.41`.

Verification state:

- Completed so far:
  - Focused baseline and accepted focused probe for
    `BM_LcpCompare/FrictionIndex/Apgd/`, with Standard APGD rows included in
    the probe.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - Focused CTest:
    `ctest --test-dir build/default/cpp/Release --output-on-failure -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' -j 1`
    passed.
- Still required before commit:
  - Run `pixi run lint`.
  - Run `git diff --check`.
- No push has been performed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks and
   commit with `Raise APGD exact gate for friction-index comparison`.
3. If this checkpoint is already committed, investigate Boxed
   `SymmetricPsor 1.57`, Boxed `RedBlackGaussSeidel 1.53`, or FrictionIndex
   `Sap 1.58` / `ShockPropagation 1.58` under the same packet-like evidence
   rules.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Jacobi Standard Gate / LLT Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 73]`.
- Last committed checkpoint:
  `343949c66f3 Use LLT for APGD and SAP standard exact paths`.
- Checkpoint target:
  `Raise Jacobi standard exact gate and use LLT`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 74 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- This is a bounded packet-like slice under the PR #2986 harness constraints:
  the accepted code change is the `JacobiSolver` Standard strict-interior exact
  path gate and helper.
- The evidence packet uses the current interim LCP identity path: benchmark
  names encode problem family, manifest solver name, and size; the profile
  script derives rows from those names plus `contract_ok` and timing fields;
  the Python demo exposes manifest metadata and native support checks.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/projection/jacobi_solver.cpp`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Current implementation slice:

- `JacobiSolver` now allows Standard strict-interior exact solves through the
  96-row comparison packet and uses
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- A gate-only 48-row probe that kept the LU helper was rejected because it did
  not materially improve the 48-row path and worsened the 24/96 focused rows.

Focused and profile evidence:

- Baseline:
  `build/standard_jacobi_gate_baseline.json`.
- Rejected gate-only probe:
  `build/standard_jacobi_gate48_probe.json`.
- Accepted focused probe:
  `build/standard_jacobi_gate96_llt_probe.json`.
- Focused Standard `Jacobi` timings moved approximately:
  - `Jacobi/12`: `825.00ns -> 781.00ns`.
  - `Jacobi/24`: `2659.00ns -> 2145.00ns`.
  - `Jacobi/48`: `10198.00ns -> 9424.00ns`.
  - `Jacobi/96`: `55301.00ns -> 40143.00ns`.
- Latest regenerated profile highlights:
  - Standard: `Jacobi 1.22`; no solver average is above `1.6x`; highest rows
    are `Baraff 1.57`, `MPRGP 1.44`, `Pgs 1.40`, and `Tgs 1.35`.
  - Boxed: no solver average is above `1.6x`; highest rows are
    `RedBlackGaussSeidel 1.51`, `ShockPropagation 1.48`,
    `SymmetricPsor 1.47`, and `NNCG 1.36`.
  - FrictionIndex: `Apgd 1.61`; next rows are `ShockPropagation 1.57`,
    `Sap 1.51`, and `Admm 1.45`.

Verification state:

- Completed so far:
  - Focused baseline, rejected gate-only probe, and accepted focused probe for
    `BM_LcpCompare/Standard/Jacobi/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - Focused CTest:
    `ctest --test-dir build/default/cpp/Release --output-on-failure -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' -j 1`
    passed.
- Still required before commit:
  - Run `pixi run lint`.
  - Run `git diff --check`.
- No push has been performed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks and
   commit with `Raise Jacobi standard exact gate and use LLT`.
3. If this checkpoint is already committed, investigate Boxed
   `RedBlackGaussSeidel 1.51`, Boxed `ShockPropagation 1.48`, or FrictionIndex
   `Apgd 1.61` under the same packet-like evidence rules.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - APGD/SAP Standard LLT Paths

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 72]`.
- Last committed checkpoint:
  `4b9ad147c09 Use LLT for RedBlackGaussSeidel standard exact path`.
- Checkpoint target:
  `Use LLT for APGD and SAP standard exact paths`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 73 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- This is a bounded packet-like slice under the PR #2986 harness constraints:
  the accepted code changes are the `ApgdSolver` and `SapSolver` Standard
  strict-interior exact paths.
- The evidence packet uses the current interim LCP identity path: benchmark
  names encode problem family, manifest solver name, and size; the profile
  script derives rows from those names plus `contract_ok` and timing fields;
  the Python demo exposes manifest metadata and native support checks.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/other/sap_solver.cpp`
- `dart/math/lcp/projection/apgd_solver.cpp`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Current implementation slice:

- `ApgdSolver` and `SapSolver` Standard strict-interior exact paths now use
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)` instead of the
  LU-only strict-interior helper.
- The APGD 96-row focused benchmark remains on the iterative path; the observed
  96-row movement is recorded as benchmark noise rather than acceptance
  evidence for the exact-path change.

Focused and profile evidence:

- Baseline:
  `build/standard_sap_apgd_baseline.json`.
- Accepted focused probe:
  `build/standard_sap_apgd_llt_probe.json`.
- Focused Standard `Apgd` timings moved approximately:
  - `Apgd/12`: `856.00ns -> 783.00ns`.
  - `Apgd/24`: `2893.00ns -> 2134.00ns`.
  - `Apgd/48`: `11232.00ns -> 8900.00ns`.
  - `Apgd/96`: `64871.00ns -> 58359.00ns`
    (iterative path; not used as acceptance evidence).
- Focused Standard `Sap` timings moved approximately:
  - `Sap/12`: `958.00ns -> 777.00ns`.
  - `Sap/24`: `2843.00ns -> 2250.00ns`.
  - `Sap/48`: `11199.00ns -> 9113.00ns`.
  - `Sap/96`: `59969.00ns -> 40996.00ns`.
- Latest regenerated profile highlights:
  - Standard: no solver average is above `1.6x`; highest rows are
    `Jacobi 1.59`, `MPRGP 1.47`, `Sap 1.46`,
    `BoxedSemiSmoothNewton 1.43`, and `Apgd 1.40`.
  - Boxed: no solver average is above `1.6x`; highest rows are
    `RedBlackGaussSeidel 1.55`, `SymmetricPsor 1.54`,
    `ShockPropagation 1.43`, `NNCG 1.39`, and `Sap 1.35`.
  - FrictionIndex: no solver average is above `1.6x`; highest rows are
    `ShockPropagation 1.54`, `Sap 1.45`, `Apgd 1.44`, and
    `BoxedSemiSmoothNewton 1.37`.

Verification state:

- Completed so far:
  - Focused baseline and accepted focused probe for
    `BM_LcpCompare/Standard/(Sap|Apgd)/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - Focused CTest:
    `ctest --test-dir build/default/cpp/Release --output-on-failure -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' -j 1`
    passed.
- Still required before commit:
  - Run `pixi run lint`.
  - Run `git diff --check`.
- No push has been performed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks and
   commit with `Use LLT for APGD and SAP standard exact paths`.
3. If this checkpoint is already committed, investigate Standard `Jacobi 1.59`
   or Boxed `RedBlackGaussSeidel 1.55` / `SymmetricPsor 1.54` under the same
   packet-like evidence rules.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - RedBlackGaussSeidel Standard LLT Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 71]`.
- Last committed checkpoint:
  `2d1ae4bd041 Use LLT for BlockedJacobi standard exact path`.
- Checkpoint target:
  `Use LLT for RedBlackGaussSeidel standard exact path`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 72 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- This is a bounded packet-like slice under the PR #2986 harness constraints:
  the only accepted code change is the `RedBlackGaussSeidelSolver` Standard
  strict-interior exact path.
- The evidence packet uses the current interim LCP identity path: benchmark
  names encode problem family, manifest solver name, and size; the profile
  script derives rows from those names plus `contract_ok` and timing fields;
  the Python demo exposes manifest metadata and native support checks.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/projection/red_black_gauss_seidel_solver.cpp`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Current implementation slice:

- `RedBlackGaussSeidelSolver` Standard strict-interior exact paths now use
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)` instead of the
  LU-only strict-interior helper.
- The 96-row focused benchmark remains on the iterative path; the observed
  96-row timing movement is recorded as benchmark noise rather than acceptance
  evidence for the exact-path change.

Focused and profile evidence:

- Baseline:
  `build/standard_redblack_baseline.json`.
- Accepted focused probe:
  `build/standard_redblack_llt_probe.json`.
- Focused Standard `RedBlackGaussSeidel` timings moved approximately:
  - `RedBlackGaussSeidel/12`: `884.00ns -> 750.00ns`.
  - `RedBlackGaussSeidel/24`: `3096.00ns -> 1936.00ns`.
  - `RedBlackGaussSeidel/48`: `11944.00ns -> 8659.00ns`.
  - `RedBlackGaussSeidel/96`: `52695.00ns -> 56005.00ns`
    (iterative path; not used as acceptance evidence).
- Latest regenerated profile highlights:
  - Standard: `RedBlackGaussSeidel 1.39`; highest rows are `Sap 1.83`,
    `Apgd 1.64`, `MPRGP 1.59`, and `Jacobi 1.50`.
  - Boxed: highest rows are `NNCG 1.72`, `SymmetricPsor 1.52`,
    `Apgd 1.52`, `ShockPropagation 1.50`, and
    `RedBlackGaussSeidel 1.48`.
  - FrictionIndex: no solver average is above `1.6x`; highest rows are
    `Apgd 1.51`, `ShockPropagation 1.51`, `Sap 1.46`, and `Admm 1.40`.

Verification state:

- Completed so far:
  - Focused baseline and accepted focused probe for
    `BM_LcpCompare/Standard/RedBlackGaussSeidel/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - Focused CTest:
    `ctest --test-dir build/default/cpp/Release --output-on-failure -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' -j 1`
    passed.
- Still required before commit:
  - Run `pixi run lint`.
  - Run `git diff --check`.
- No push has been performed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks and
   commit with `Use LLT for RedBlackGaussSeidel standard exact path`.
3. If this checkpoint is already committed, investigate Standard `Sap 1.83` or
   Standard `Apgd 1.64` under the same packet-like evidence rules, or choose
   Boxed `NNCG 1.72`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - BlockedJacobi Standard LLT Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 70]`.
- Last committed checkpoint:
  `f7a3a94dc09 Record PLAN-091 harness alignment for LCP work`.
- Checkpoint target:
  `Use LLT for BlockedJacobi standard exact path`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 71 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

DART 7 harness alignment:

- This is a bounded packet-like slice under the PR #2986 harness constraints:
  the only accepted code change is the `BlockedJacobiSolver` Standard
  strict-interior exact path.
- The current evidence packet uses the interim LCP identity path documented in
  the previous section: `BM_LcpCompare` benchmark names encode problem family,
  manifest solver name, and size; the profile script derives rows from those
  names plus `contract_ok` and timing fields; the Python demo exposes manifest
  metadata and native support checks.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/projection/blocked_jacobi_solver.cpp`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Current implementation slice:

- `BlockedJacobiSolver` Standard strict-interior exact paths now use
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)` instead of the
  LU-only strict-interior helper.
- A combined `Jacobi` + `BlockedJacobi` LLT-first probe was rejected for
  `Jacobi`: it improved 24-row timing but regressed the 12-row focused row, so
  `JacobiSolver` remains on the existing helper for this checkpoint.

Focused and profile evidence:

- Baseline:
  `build/standard_jacobi_blocked_baseline.json`.
- Rejected combined probe:
  `build/standard_jacobi_blocked_llt_probe.json`.
- Accepted focused probe:
  `build/standard_blocked_jacobi_llt_probe.json`.
- Focused Standard `BlockedJacobi` timings moved approximately:
  - `BlockedJacobi/12`: `827.32ns -> 816.27ns`.
  - `BlockedJacobi/24`: `2887.44ns -> 2252.69ns`.
  - `BlockedJacobi/48`: `11828.52ns -> 9120.27ns`.
  - `BlockedJacobi/96`: `63323.76ns -> 40585.69ns`.
- Latest regenerated profile highlights:
  - Standard: `BlockedJacobi 1.18`; highest rows are `Jacobi 1.66`,
    `RedBlackGaussSeidel 1.63`, `Apgd 1.56`, and `Sap 1.51`.
  - Boxed: highest rows are `Sap 1.68`, `RedBlackGaussSeidel 1.53`,
    `SymmetricPsor 1.50`, and `ShockPropagation 1.48`.
  - FrictionIndex: highest rows are `ShockPropagation 1.65`, `Sap 1.64`,
    `Apgd 1.59`, `NNCG 1.53`, and `SubspaceMinimization 1.52`.

Verification state:

- Completed so far:
  - Focused baseline, rejected combined probe, and accepted focused probe for
    `BM_LcpCompare/Standard/(Jacobi|BlockedJacobi)/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - Focused CTest:
    `ctest --test-dir build/default/cpp/Release --output-on-failure -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' -j 1`
    passed.
- Still required before commit:
  - Run `pixi run lint`.
  - Run `git diff --check`.
- No push has been performed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks and
   commit with `Use LLT for BlockedJacobi standard exact path`.
3. If this checkpoint is already committed, investigate Standard `Jacobi 1.66`
   or Standard `RedBlackGaussSeidel 1.63` under the same packet-like evidence
   rules, or choose Boxed `Sap 1.68`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Post-PLAN-091 Merge / Harness Alignment

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 69]`.
- Latest local commit:
  `b6b1b12544e Merge remote-tracking branch 'origin/main' into feature/lcp-solver-interface-demos`.
- Latest LCP implementation checkpoint:
  `d35a3ffa099 Use LLT for Lemke and Baraff standard exact paths`.
- The latest `origin/main` was fetched via HTTPS because the SSH remote was not
  reachable from this environment, then merged locally.
- The merged `origin/main` includes PR #2986:
  `bb851f45360 Add DART 7 architecture assessment, PLAN-091 hardening plan, and work-packet harness (#2986)`.
- This branch has not been pushed after the merge. Do not push, open a PR, or
  mutate GitHub state without explicit maintainer/user approval.

DART 7 harness alignment from PR #2986:

- `docs/ai/orchestration.md` now defines the packet executor model. For this
  existing LCP task, keep future slices bounded like work packets: one
  objective, explicit scope/non-goals, concrete acceptance evidence, and clear
  handoff notes.
- `docs/design/dart7_architecture_assessment.md` identifies apples-to-apples
  solver comparison substrate and internal solver contracts as load-bearing DART
  7 gaps.
- `docs/plans/solver-family-intake.md` now requires solver-family work to
  record how it enters the internal solver contract and requires benchmark or
  evidence packets to machine-record the resolved solver configuration.
- `docs/plans/091-architecture-hardening.md` is now the standing hardening plan
  for those contract/identity gaps. This LCP work predates PLAN-091, so do not
  invent a new schema inside this dev-task. Instead, keep every new LCP evidence
  packet explicit about the current interim identity mechanism and leave any
  durable schema migration to PLAN-091-aligned work.

Current LCP identity/evidence stance:

- `BM_LcpCompare` rows currently encode the problem family, manifest solver
  name, and benchmark size in the benchmark name.
- `scripts/lcp_performance_profile.py` derives profile rows from those benchmark
  names plus `contract_ok` and timing fields.
- `python/examples/demos/scenes/lcp_physics.py` exposes solver manifest metadata
  and native support checks for the demo panels.
- Until a PLAN-091 resolved-identity schema lands, handoff notes and profile
  updates must explicitly state this interim identity path instead of implying
  that full DART 7 resolved solver identity is already implemented.

Latest verified checkpoint before the merge:

- `d35a3ffa099` moved `LemkeSolver` and `BaraffSolver` Standard
  strict-interior exact paths to
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)`.
- The regenerated full profile after that checkpoint showed the largest
  remaining rows as:
  - Standard: `Jacobi 1.64`, `BlockedJacobi 1.62`, `Apgd 1.59`,
    `RedBlackGaussSeidel 1.57`, and `Sap 1.57`.
  - Boxed: `NNCG 1.71`, `RedBlackGaussSeidel 1.66`,
    `SymmetricPsor 1.63`, `ShockPropagation 1.59`,
    `BoxedSemiSmoothNewton 1.55`, and `SubspaceMinimization 1.51`.
  - FrictionIndex: `ShockPropagation 1.71`, `Admm 1.59`, `NNCG 1.58`,
    `Sap 1.56`, and `Apgd 1.53`.
- Pre-merge verification for that checkpoint passed:
  - Focused Python demo metadata test.
  - CSV shape check.
  - Focused C++ build and CTest for LCP validation/solver coverage.
  - `pixi run build`.
  - `pixi run test-unit` (`161/161`).
  - `pixi run lint`.
  - `git diff --check`.

Immediate resume guidance:

1. Start with `git status -sb` and confirm the latest commit is the
   `origin/main` merge commit above.
2. Treat the PR #2986 harness docs as active constraints for any further solver
   performance/evidence slice.
3. If continuing optimization, start with a focused baseline for Standard
   `Jacobi` and `BlockedJacobi`, or choose Boxed `NNCG`; record the interim
   resolved-identity path in this dev-task for any new evidence packet.
4. Before any checkpoint commit, run `pixi run lint` as required by
   `AGENTS.md`. Run broader build/test gates when code or behavior changes.
5. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Lemke/Baraff Standard LLT Paths

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 75]`.
- Last committed checkpoint:
  `e60b8563366 Use LLT for SubspaceMinimization standard exact path`.
- Checkpoint target:
  `Use LLT for Lemke and Baraff standard exact paths`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 76 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.
- Latest continuation resumed after the hand-off-only stop request and is
  closing this checkpoint locally.

Current dirty files before commit:

- `CHANGELOG.md`
- `dart/math/lcp/pivoting/baraff_solver.cpp`
- `dart/math/lcp/pivoting/lemke_solver.cpp`
- `docs/background/lcp/03_pivoting-methods.md`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `tests/unit/math/lcp/test_additional_solvers.cpp`
- `tests/unit/math/lcp/test_lcp_generated_coverage.cpp`

Current implementation slice:

- `LemkeSolver` and `BaraffSolver` standard strict-interior exact paths now use
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)` instead of the
  LU-only strict-interior helper.
- Existing pivot/barrier exact-path unit coverage exercises the same accepted
  strict-interior solve route.
- The regenerated full profile moves Standard `Baraff` to `1.15x` and Standard
  `Lemke` to `1.25x`. The largest remaining Standard rows are now
  `Jacobi 1.64`, `BlockedJacobi 1.62`, `Apgd 1.59`,
  `RedBlackGaussSeidel 1.57`, and `Sap 1.57`.

Focused and profile evidence:

- Baseline:
  `build/standard_lemke_baraff_baseline.json`.
- Accepted focused probe:
  `build/standard_lemke_baraff_llt_probe.json`.
- Focused Standard Baraff timings moved approximately:
  - `Baraff/12`: `893.58ns -> 855.07ns`.
  - `Baraff/24`: `3105.49ns -> 2351.73ns`.
  - `Baraff/48`: `12212.39ns -> 9582.00ns`.
  - `Baraff/96`: `63965.92ns -> 42414.44ns`.
- Focused Standard Lemke timings moved approximately:
  - `Lemke/12`: `908.48ns -> 724.84ns`.
  - `Lemke/24`: `3315.46ns -> 2169.25ns`.
  - `Lemke/48`: `12320.24ns -> 9214.86ns`.
  - `Lemke/96`: `64622.48ns -> 43291.53ns`.
- Latest regenerated profile highlights:
  - Standard: `Baraff 1.15`, `Lemke 1.25`; highest rows are
    `Jacobi 1.64`, `BlockedJacobi 1.62`, `Apgd 1.59`,
    `RedBlackGaussSeidel 1.57`, and `Sap 1.57`.
  - Boxed: highest rows are `NNCG 1.71`, `RedBlackGaussSeidel 1.66`,
    `SymmetricPsor 1.63`, `ShockPropagation 1.59`,
    `BoxedSemiSmoothNewton 1.55`, and `SubspaceMinimization 1.51`.
  - FrictionIndex: highest rows are `ShockPropagation 1.71`, `Admm 1.59`,
    `NNCG 1.58`, `Sap 1.56`, and `Apgd 1.53`.

Rejected probes from this continuation:

- Boxed `SymmetricPsor` projected-active-set exact helper:
  `build/boxed_symmetric_psor_baseline.json` vs
  `build/boxed_symmetric_psor_exact_probe.json`; it was slightly faster at 12
  rows but slower at 24 and 48 rows.
- Boxed `SymmetricPsor` relaxation sweep:
  `build/boxed_symmetric_psor_relaxation_sweep.json`; the existing
  `relaxation=1.0` row remained the best tracked boxed sweep point.
- Boxed `RedBlackGaussSeidel` projected-active-set exact helper:
  `build/boxed_redblack_baseline.json`,
  `build/boxed_redblack_exact_probe.json`, and
  `build/boxed_redblack_gate24_probe.json`; focused 12/24-row gains did not
  survive the aggregate profile, so the probe was reverted.

Verification state:

- Completed so far:
  - Focused baseline and LLT probe for
    `BM_LcpCompare/Standard/(Lemke|Baraff)/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Fixed stale Python demo profile metadata and assertions:
    removed `MPRGP` from the Standard `current_laggards` text, aligned Boxed
    `SubspaceMinimization` with the laggard text, and aligned FrictionIndex
    `Admm` with the refreshed next-largest-row text.
  - Focused Python demo metadata test:
    `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
    passed.
  - CSV shape check: `Boxed 15/200`, `FrictionIndex 16/200`,
    `Standard 23/200`.
  - `pixi run build` passed with `JOBS=5`.
  - Focused rebuild and CTest for
    `UNIT_math_lcp_math_lcp_additional_solvers` and
    `UNIT_math_lcp_math_lcp_lcp_generated_coverage` passed after aligning
    tests with exact-fast-path behavior.
  - `pixi run test-unit` passed: `161/161` tests.
  - `pixi run lint` passed with `JOBS=5`.
  - `git diff --check` passed.
- Resolved failures seen during this continuation:
  - `PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata -q`
  - Initial failure: the Standard demo metadata still said `MPRGP` is a
    next-largest row, while the test asserted `MPRGP` should not appear in that
    text.
  - `pixi run test-unit` initially failed in
    `UNIT_math_lcp_math_lcp_additional_solvers` and
    `UNIT_math_lcp_math_lcp_lcp_generated_coverage`; the max-iteration tests
    now warm-start the iterative path to avoid the cold exact fast paths, and
    the near-singular boxed generated case now treats residual and
    complementarity as the contract instead of over-constraining the expected
    vector.
- Still required before commit:
  - Create the local checkpoint commit.
- No push has been performed.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks and
   commit with
   `Use LLT for Lemke and Baraff standard exact paths`.
3. If this checkpoint is already committed, investigate Standard `Jacobi 1.64`
   and Standard `BlockedJacobi 1.62`, or Boxed `NNCG 1.71`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - SubspaceMinimization Standard LLT Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship before this checkpoint:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 74]`.
- Last committed checkpoint:
  `7d53d6a16d7 Use LLT for Dantzig and Symmetric PSOR exact paths`.
- Checkpoint target:
  `Use LLT for SubspaceMinimization standard exact path`.
- Pre-commit state: this slice is uncommitted. After this checkpoint is
  committed, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 75 commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Current implementation slice:

- `SubspaceMinimizationSolver` standard strict-interior exact path now uses
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)` instead of the
  LU-only strict-interior helper.
- Unit coverage adds `SubspaceMinimizationSolver` to the 96-size Standard
  exact-path regression with `maxIterations = 1` and zero expected iterations.
- The regenerated full profile moves Standard `SubspaceMinimization` from the
  largest remaining Standard row to `1.19x`; no Standard solver average is now
  above `1.6x`.

Focused and profile evidence:

- Baseline:
  `build/standard_subspace_baseline.json`.
- Accepted focused probe:
  `build/standard_subspace_llt_probe.json`.
- Focused Standard SubspaceMinimization timings moved approximately:
  - `SubspaceMinimization/12`: `1999.23ns -> 952.73ns`.
  - `SubspaceMinimization/24`: `5719.05ns -> 2434.37ns`.
  - `SubspaceMinimization/48`: `23889.67ns -> 12128.22ns`.
  - `SubspaceMinimization/96`: `149798.16ns -> 48358.45ns`.
- Latest regenerated profile highlights:
  - Standard: no solver above `1.6x`; highest rows are `Baraff 1.59`,
    `BlockedJacobi 1.59`, `Lemke 1.56`, `RedBlackGaussSeidel 1.54`,
    `Sap 1.53`, `Apgd 1.49`, and `Jacobi 1.48`.
  - Boxed: `SymmetricPsor 2.08` is now the largest row; next rows are
    `RedBlackGaussSeidel 1.68`, `Jacobi 1.54`, `ShockPropagation 1.45`,
    `Apgd 1.41`, and `BlockedJacobi 1.39`.
  - FrictionIndex: no solver above `1.6x`; highest rows are
    `SubspaceMinimization 1.57`, `ShockPropagation 1.51`, `NNCG 1.51`,
    `Apgd 1.50`, `RedBlackGaussSeidel 1.47`, and `Sap 1.42`.

Verification state:

- Completed so far:
  - Focused baseline and LLT probe for
    `BM_LcpCompare/Standard/SubspaceMinimization/`.
  - Focused C++ build for `BM_LCP_COMPARE` and
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Focused CTest for
    `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`.
  - Full profile regeneration into `docs/background/lcp/figures`.
  - Focused Python demo metadata test.
  - CSV shape check: `Boxed 15/200`, `FrictionIndex 16/200`,
    `Standard 23/200`.
- Final pre-commit checks after any final edits:
  - `pixi run lint`.
  - `git diff --check`.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run final lint/diff checks after
   any edits and commit with
   `Use LLT for SubspaceMinimization standard exact path`.
3. If this checkpoint is already committed, investigate Boxed
   `SymmetricPsor 2.08`, then Boxed `RedBlackGaussSeidel 1.68` or
   FrictionIndex `SubspaceMinimization 1.57`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Dantzig/Symmetric PSOR Standard LLT Paths

This is the latest hand-off state after resuming the previously stopped
Dantzig/Symmetric PSOR slice. Sections below are historical checkpoints and may
describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Local branch relationship:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 73]`.
- Last committed checkpoint:
  `ce7dbf7aa58 Raise ADMM friction exact gate`.
- Checkpoint target:
  `Use LLT for Dantzig and Symmetric PSOR exact paths`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 73 commits, with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 74
  commits.
- This current slice has not been pushed. No PR is associated with this branch
  yet.
- Do not push, open a PR, or mutate GitHub state without explicit
  maintainer/user approval.

Current dirty files:

- `dart/math/lcp/pivoting/dantzig_solver.cpp`
- `dart/math/lcp/projection/symmetric_psor_solver.cpp`
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`
- `docs/background/lcp/03_pivoting-methods.md`
- `docs/background/lcp/04_projection-methods.md`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`
- `CHANGELOG.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`

Current implementation slice:

- `DantzigSolver` standard strict-interior exact path now uses
  `detail::trySolveStrictInteriorStandardLcpLltFirst(...)` instead of the
  LU-only strict-interior helper.
- `SymmetricPsorSolver` standard strict-interior exact path now uses the same
  LLT-first helper and raises its strict-interior fast-path gate from 48 to 96
  variables, covering the current 96-row Standard comparison packet.
- Unit coverage adds `SymmetricPsorSolver` to
  `StandardStrictInteriorFastPath.HighOverheadSolversUseLargeLinearSolve` with
  a 96-size Standard problem, `maxIterations = 1`, and an expectation of zero
  iterations.
- Background docs and demo metadata have been partially updated for the latest
  regenerated profile.

Known unfinished items for this slice:

- No known focused validation failures remain after the metadata assertion fix.
- If this checkpoint is already committed, continue with the refreshed profile's
  highest remaining rows: Standard `SubspaceMinimization 1.64`, Standard
  `Baraff/Jacobi/Sap 1.57`, Standard `Lemke 1.54`, Boxed
  `ShockPropagation 1.51`, or FrictionIndex `ShockPropagation 1.53`.

Focused and profile evidence already collected before the stop request:

- Baseline:
  `build/standard_dantzig_sympsor_baseline.json`.
- Accepted focused probe:
  `build/standard_dantzig_sympsor_llt_probe.json`.
- Focused Standard Dantzig timings moved approximately:
  - `Dantzig/12`: `848.38ns -> 784.74ns`.
  - `Dantzig/24`: `2751.88ns -> 1959.32ns`.
  - `Dantzig/48`: `11071.67ns -> 8444.85ns`.
  - `Dantzig/96`: `59370.01ns -> 39535.71ns`.
- Focused Standard Symmetric PSOR timings moved approximately:
  - `SymmetricPsor/12`: `991.57ns -> 878.19ns`.
  - `SymmetricPsor/24`: `3454.77ns -> 2760.61ns`.
  - `SymmetricPsor/48`: `12911.97ns -> 10933.11ns`.
  - `SymmetricPsor/96`: `51214.00ns -> 45922.66ns`.
- Latest regenerated profile highlights from the current uncommitted slice:
  - Standard: `Dantzig 1.02`, `SymmetricPsor 1.26`,
    `SubspaceMinimization 1.64`, `Baraff/Jacobi/Sap 1.57`,
    `Lemke 1.54`, `BlockedJacobi/RedBlackGaussSeidel 1.51`,
    and `Apgd 1.50`.
  - Boxed: no solver above `1.6x`; highest rows are
    `ShockPropagation 1.51`, `SymmetricPsor 1.47`,
    `RedBlackGaussSeidel 1.47`, `BGS 1.36`, and `BlockedJacobi 1.36`.
  - FrictionIndex: no solver above `1.6x`; highest rows are
    `ShockPropagation 1.53`, `Sap 1.44`, `Apgd 1.43`,
    `Admm 1.37`, and `SymmetricPsor 1.36`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
```

The CSV shape check reported `Boxed 15/200`, `FrictionIndex 16/200`, and
`Standard 23/200`. Run `git diff --check` after any final edits before
committing.

Immediate resume guidance:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, run `git diff --check` after any
   final edits and commit with
   `Use LLT for Dantzig and Symmetric PSOR exact paths`.
3. If this checkpoint is already committed, continue with Standard
   `SubspaceMinimization 1.64`, Standard `Baraff/Jacobi/Sap 1.57`, Standard
   `Lemke 1.54`, Boxed `ShockPropagation 1.51`, or FrictionIndex
   `ShockPropagation 1.53`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - ADMM FrictionIndex Exact Gate

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `7dad05ea3aa Raise BGS friction exact gate`.
- Current checkpoint target:
  `Raise ADMM friction exact gate`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 72 commits, with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 73
  commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.

Current implementation slice:

- `AdmmSolver` raises its strict-interior friction-index exact gate from 12 to
  48 variables. This covers the current 16-contact FrictionIndex comparison
  packet while leaving the large 64-contact row on the operator-splitting path.
- A 192-variable probe was rejected because it helped the medium row but did
  not materially improve the large row and had worse regression risk.
- Unit coverage adds a 16-contact FrictionIndex ADMM exact-path regression with
  `maxIterations = 1` and expects zero iterations.
- The regenerated full profile moved FrictionIndex `Admm` from `1.70x` to
  `1.41x`. The largest remaining rows are now Standard `Dantzig 1.73`,
  Standard `SymmetricPsor 1.71`, Standard `Baraff 1.61`, FrictionIndex
  `Apgd 1.57`, and FrictionIndex `ShockPropagation 1.55`.

Focused and profile evidence:

- Baseline:
  `build/friction_index_admm_baseline.json`.
- Rejected large gate probe:
  `build/friction_index_admm_gate192_probe.json`.
- Accepted medium gate probe:
  `build/friction_index_admm_gate48_probe.json`.
- Focused FrictionIndex timings moved approximately:
  - `Admm/4`: `1103.20ns -> 1151.38ns`.
  - `Admm/16`: `19081.82ns -> 12127.16ns`.
  - `Admm/64`: `281190.62ns -> 283267.40ns`.
- Latest regenerated profile highlights:
  - Standard: `Dantzig 1.73`, `SymmetricPsor 1.71`, `Baraff 1.61`,
    `Apgd 1.56`, with `Admm 1.09`.
  - Boxed: no solver above `1.5x`; highest rows are
    `SymmetricPsor 1.46`, `ShockPropagation 1.43`, and
    `RedBlackGaussSeidel 1.42`.
  - FrictionIndex: no solver above `1.6x`; `Admm 1.41`; highest rows are
    `Apgd 1.57`, `ShockPropagation 1.55`, and `Sap 1.46`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
git diff --check
```

Immediate resume guidance:

1. Inspect `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, review the expected dirty files and
   commit with `Raise ADMM friction exact gate`.
3. If this checkpoint is already committed, continue with Standard
   `Dantzig 1.73`, Standard `SymmetricPsor 1.71`, Standard `Baraff 1.61`,
   FrictionIndex `Apgd 1.57`, or FrictionIndex `ShockPropagation 1.55`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - BGS FrictionIndex Exact Gate

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `f27dad169a7 Use LLT for boxed active-set exact helper`.
- Current checkpoint target:
  `Raise BGS friction exact gate`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 71 commits, with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 72
  commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.

Current implementation slice:

- `BgsSolver` raises its strict-interior exact gate from 96 to 192 variables so
  the current 64-contact FrictionIndex comparison packet exits through the
  shared LLT-first friction exact helper.
- Unit coverage adds a 64-contact FrictionIndex BGS exact-path regression with
  `maxIterations = 1` and expects zero iterations.
- The regenerated full profile moved FrictionIndex `BGS` from the largest row
  (`1.65x` before this slice) to `1.24x`. The largest remaining rows are now
  FrictionIndex `Admm 1.70`, Boxed `SymmetricPsor 1.62`, FrictionIndex
  `ShockPropagation 1.57`, FrictionIndex `Sap 1.55`, and Standard
  `Lemke 1.54`.

Focused and profile evidence:

- Baseline:
  `build/friction_index_bgs_baseline.json`.
- Accepted focused probe:
  `build/friction_index_bgs_gate192_probe.json`.
- Focused FrictionIndex timings moved approximately:
  - `BGS/4`: `1629.88ns -> 1179.10ns`.
  - `BGS/16`: `12570.26ns -> 12323.68ns`.
  - `BGS/64`: `330609.46ns -> 254061.50ns`.
- Latest regenerated profile highlights:
  - Standard: no solver above `1.54x`; highest rows are `Lemke 1.54` and
    `Jacobi 1.53`.
  - Boxed: `SymmetricPsor 1.62` is the only solver above `1.6x`;
    `RedBlackGaussSeidel 1.51` and `ShockPropagation 1.51` are next.
  - FrictionIndex: `BGS 1.24`; highest rows are `Admm 1.70`,
    `ShockPropagation 1.57`, `Sap 1.55`, and `Apgd 1.42`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
git diff --check
```

Immediate resume guidance:

1. Inspect `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, review the expected dirty files and
   commit with `Raise BGS friction exact gate`.
3. If this checkpoint is already committed, continue with FrictionIndex
   `Admm 1.70`, Boxed `SymmetricPsor 1.62`, FrictionIndex
   `ShockPropagation 1.57`, FrictionIndex `Sap 1.55`, or Standard
   `Lemke 1.54`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed Exact Helper LLT Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Checkpoint note: this section records the boxed-helper LLT slice and its
focused validation evidence. The checkpoint commit should be titled
`Use LLT for boxed active-set exact helper`.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `13af6a5463c Use LLT for standard BGS and NNCG exact paths`.
- Current checkpoint target:
  `Use LLT for boxed active-set exact helper`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 70 commits, with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 71
  commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.
- Pre-commit worktree for this checkpoint contained the boxed-helper slice and
  its generated docs/demo/profile metadata updates. Expected modified files
  before the checkpoint commit were `CHANGELOG.md`,
  `dart/math/lcp/lcp_validation.hpp`,
  `docs/background/lcp/04_projection-methods.md`,
  `docs/background/lcp/figures/performance_profile_boxed.csv`,
  `docs/background/lcp/figures/performance_profile_frictionindex.csv`,
  `docs/background/lcp/figures/performance_profile_standard.csv`,
  `docs/dev_tasks/lcp_solver_interface_demos/README.md`,
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`,
  `python/examples/demos/scenes/lcp_physics.py`, and
  `python/tests/unit/test_py_demo_panels.py`.

Current implementation slice:

- The shared projected-active-set boxed-LCP exact helper now tries LLT-based
  dense solves for the unconstrained and reduced free-row systems and falls
  back to the previous LU path if the LLT-based active-set candidate fails
  validation.
- The accepted path preserves the previous active-set validation contract while
  reducing dense-solve cost for the current SPD boxed comparison packets.
- The regenerated full profile now reports no Boxed solver above `1.51x`
  average; the previous Boxed `ShockPropagation` and `BGS` above-`2x` rows are
  gone.

Focused and profile evidence:

- Baseline:
  `build/boxed_exact_helper_baseline.json`.
- Accepted focused probe:
  `build/boxed_exact_helper_llt_probe.json`.
- Focused Boxed timings moved approximately:
  - `BGS/24`: `4004.38ns -> 3838.84ns`, `iterations 0 -> 0`.
  - `BGS/48`: `16886.30ns -> 13481.80ns`, `iterations 0 -> 0`.
  - `ShockPropagation/24`: `4153.02ns -> 3947.97ns`, `iterations 0 -> 0`.
  - `ShockPropagation/48`: `15515.32ns -> 13510.71ns`, `iterations 0 -> 0`.
  - `Dantzig/24`: `4368.56ns -> 2835.80ns`, `iterations 0 -> 0`.
  - `Sap/48`: `14455.75ns -> 11305.99ns`, `iterations 0 -> 0`.
- Latest regenerated profile highlights:
  - Standard: no solver above `1.55x`; highest row is `Sap 1.55`.
  - Boxed: no solver above `1.51x`; highest rows are
    `RedBlackGaussSeidel 1.51` and `ShockPropagation 1.50`.
  - FrictionIndex: no solver above `2x`; highest rows are `BGS 1.65`,
    `BoxedSemiSmoothNewton 1.52`, `Admm 1.50`, and
    `ShockPropagation 1.50`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS \
  pixi run lint
git diff --check
```

CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
columns.

Immediate resume guidance:

1. Inspect `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, review the expected dirty files
   listed above, run the required pre-commit checks, and commit with
   `Use LLT for boxed active-set exact helper`.
3. If this checkpoint is already committed, continue with FrictionIndex
   `BGS 1.65`, FrictionIndex `BoxedSemiSmoothNewton 1.52`, or Standard
   `Sap 1.55`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Standard BGS/NNCG LLT Exact Paths

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `b57e03cdee1 Use LLT for standard Newton exact paths`.
- Current checkpoint target:
  `Use LLT for standard BGS and NNCG exact paths`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 69 commits, with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 70
  commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.

Current implementation slice:

- `NncgSolver` now uses the validated LLT-first strict-interior Standard exact
  helper for default non-warm-started Standard rows.
- `BgsSolver` now uses the same LLT-first Standard exact helper and raises its
  Standard exact gate to 96 variables, covering the current 96-row Standard
  comparison packet.
- Unit coverage adds a 96-row Standard exact-path regression for BGS and NNCG.
- The full profile was regenerated and now reports every Standard solver at or
  below `1.50x` average. The next profile-driven targets are Boxed
  `ShockPropagation 2.18` and Boxed `BGS 2.12`.

Focused and profile evidence:

- Baseline:
  `build/standard_nncg_bgs_baseline.json`.
- Accepted focused probe:
  `build/standard_nncg_bgs_llt_probe.json`.
- Focused Standard timings moved approximately:
  - `BGS/12`: `917.19ns -> 865.36ns`, `iterations 0 -> 0`.
  - `BGS/24`: `3300.26ns -> 2380.74ns`, `iterations 0 -> 0`.
  - `BGS/48`: `13747.08ns -> 9523.33ns`, `iterations 0 -> 0`.
  - `BGS/96`: `74648.65ns -> 42148.97ns`, `iterations 5 -> 0`.
  - `NNCG/12`: `1153.89ns -> 875.81ns`, `iterations 0 -> 0`.
  - `NNCG/24`: `3267.90ns -> 2443.98ns`, `iterations 0 -> 0`.
  - `NNCG/48`: `12084.66ns -> 10303.36ns`, `iterations 0 -> 0`.
  - `NNCG/96`: `63547.04ns -> 45197.60ns`, `iterations 0 -> 0`.
- Latest regenerated profile highlights:
  - Standard: `BGS 1.12`, `NNCG 1.17`, and no Standard solver above `1.50x`.
  - Boxed: `ShockPropagation 2.18` and `BGS 2.12` are above `2x`.
  - FrictionIndex: no solver above `2x`; highest rows are `Admm 1.62`,
    `ShockPropagation 1.55`, and `Sap 1.50`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
columns.

Immediate resume guidance:

1. Inspect `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, review the BGS/NNCG slice and
   checkpoint it with `Use LLT for standard BGS and NNCG exact paths`.
3. If this checkpoint is already committed, continue with Boxed
   `ShockPropagation 2.18` and Boxed `BGS 2.12`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Standard LLT Exact Paths

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `a1f83f6fe43 Extend Jacobi and RedBlackGaussSeidel friction exact paths`.
- Current checkpoint target:
  `Use LLT for standard Newton exact paths`.
- Pre-commit state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 68 commits, with the current
  Standard LLT slice uncommitted. After this checkpoint is committed, it should
  be ahead by 69 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.

Current uncommitted implementation slice:

- `dart/math/lcp/lcp_validation.hpp` now keeps the default
  `trySolveStrictInteriorStandardLcp(...)` on the prior LU-based path and adds
  an LLT-first variant, `trySolveStrictInteriorStandardLcpLltFirst(...)`, with
  the same validated acceptance checks and LU fallback.
- The broad shared-helper LLT attempt was intentionally narrowed because it
  helped dense Newton/interior-point Standard rows but risked changing
  low-overhead exact-path users. The default helper therefore remains the
  conservative LU helper.
- The dense Standard fast paths in the following solvers call the new LLT-first
  helper:
  `InteriorPointSolver`, `FischerBurmeisterNewtonSolver`,
  `MinimumMapNewtonSolver`, `PenalizedFischerBurmeisterNewtonSolver`, and
  `BoxedSemiSmoothNewtonSolver`.
- The regenerated profile CSVs, Python demo summary strings, Python metadata
  assertions, projection-method docs, changelog, and this hand-off were
  refreshed.

Files changed by this checkpoint:

- `CHANGELOG.md`
- `dart/math/lcp/lcp_validation.hpp`
- `dart/math/lcp/other/interior_point_solver.cpp`
- `dart/math/lcp/newton/fischer_burmeister_newton_solver.cpp`
- `dart/math/lcp/newton/minimum_map_newton_solver.cpp`
- `dart/math/lcp/newton/penalized_fischer_burmeister_newton_solver.cpp`
- `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp`
- `docs/background/lcp/04_projection-methods.md`
- `docs/background/lcp/figures/performance_profile_standard.csv`
- `docs/background/lcp/figures/performance_profile_boxed.csv`
- `docs/background/lcp/figures/performance_profile_frictionindex.csv`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `python/examples/demos/scenes/lcp_physics.py`
- `python/tests/unit/test_py_demo_panels.py`

Profile evidence gathered before final verification:

- Baseline before the Standard LLT work:
  `build/standard_interior_penalized_baseline.json`.
- Broad shared-helper LLT probe, not the final narrowed implementation:
  `build/standard_llt_exact_probe.json`.
- Accepted narrowed focused probe:
  `build/standard_newton_llt_exact_probe.json`.
- The full profile was regenerated into `build/lcp_profile_full.json` and
  `docs/background/lcp/figures/`.

Focused Standard timing evidence from the narrowed probe:

- `InteriorPoint/24`: `3219.35ns -> 2450.15ns`.
- `InteriorPoint/48`: `13020.26ns -> 10452.61ns`.
- `InteriorPoint/96`: `81475.85ns -> 46214.64ns`.
- `PenalizedFischerBurmeisterNewton/24`: `3268.03ns -> 2546.23ns`.
- `PenalizedFischerBurmeisterNewton/96`: `66040.66ns -> 47382.26ns`.
- `FischerBurmeisterNewton/24`: `3278.38ns -> 2452.76ns`.
- `FischerBurmeisterNewton/96`: `66017.55ns -> 47803.73ns`.
- `MinimumMapNewton/24`: `3352.45ns -> 2467.01ns`.
- `MinimumMapNewton/96`: `67322.47ns -> 46319.16ns`.

Latest regenerated profile highlights:

- Standard high rows after the LLT-first narrowing:
  `NNCG 2.00`, `BGS 1.97`, `SubspaceMinimization 1.62`,
  `Baraff 1.58`, `Lemke 1.56`, `Dantzig 1.49`, `Apgd 1.46`,
  `Jacobi 1.46`, `RedBlackGaussSeidel 1.46`, and `Sap 1.43`.
- Standard dense Newton/interior-point rows are now around:
  `FischerBurmeisterNewton 1.17`, `InteriorPoint 1.17`,
  `PenalizedFischerBurmeisterNewton 1.17`,
  `BoxedSemiSmoothNewton 1.16`, and `MinimumMapNewton 1.25`.
- Boxed high rows:
  `BGS 1.70`, `ShockPropagation 1.67`,
  `BoxedSemiSmoothNewton 1.65`, `Sap 1.65`, `NNCG 1.63`,
  and `Apgd 1.62`.
- FrictionIndex high rows:
  `Admm 1.64`, `Apgd 1.63`, `ShockPropagation 1.62`,
  `BGS 1.60`, `Sap 1.50`, and `NNCG 1.47`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
columns.

Immediate resume guidance for a fresh session:

1. Start with `git status -sb` and `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, review the Standard LLT slice and
   checkpoint it with `Use LLT for standard Newton exact paths`.
3. If this checkpoint is already committed, continue with Standard `NNCG 2.00`,
   Standard `BGS 1.97`, Boxed `BGS 1.70`, Boxed `ShockPropagation 1.67`, or
   FrictionIndex `Admm 1.64`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Jacobi/RBGS FrictionIndex Exact Paths

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `d00106704aa Optimize friction exact helper with LLT`.
- Current checkpoint target:
  `Extend Jacobi and RedBlackGaussSeidel friction exact paths`.
- Pre-commit state: the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 67 commits with this slice
  uncommitted. After this checkpoint is committed, it should be ahead by 68
  commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.

Current implementation slice:

- `JacobiSolver` and `RedBlackGaussSeidelSolver` now try the shared validated
  LLT-first strict-interior friction-index exact helper for default,
  non-warm-started solves up to 192 variables, covering the current
  64-contact FrictionIndex comparison packet.
- Their standard-LCP exact gates are unchanged, and boxed paths remain
  iterative because the current boxed profile does not justify adding dense
  boxed exact probes to these solvers.
- Unit coverage adds both solvers to the small FrictionIndex exact-path smoke
  coverage and adds large 64-contact exact-path regressions for each solver.
- The profile CSVs in `docs/background/lcp/figures/`, Python demo summary
  strings, Python metadata assertions, projection-method docs, changelog, and
  this hand-off were refreshed.

Focused and profile evidence:

- Baseline:
  `build/friction_index_rbg_jacobi_baseline.json`.
- Accepted focused probe:
  `build/friction_index_rbg_jacobi_exact_probe.json`.
- Focused FrictionIndex timings moved approximately:
  - `Jacobi/4`: `1748.16ns -> 1089.70ns`, `iterations 8 -> 0`.
  - `Jacobi/16`: `12519.10ns -> 11009.86ns`, `iterations 12 -> 0`.
  - `Jacobi/64`: `356776.39ns -> 236860.91ns`, `iterations 17 -> 0`.
  - `RedBlackGaussSeidel/4`: `1533.68ns -> 1047.22ns`,
    `iterations 5 -> 0`.
  - `RedBlackGaussSeidel/16`: `16208.84ns -> 12713.89ns`,
    `iterations 8 -> 0`.
  - `RedBlackGaussSeidel/64`: `295732.70ns -> 253038.00ns`,
    `iterations 7 -> 0`.
- Regenerated `build/lcp_profile_full.json` reports FrictionIndex
  `Jacobi 1.23` and `RedBlackGaussSeidel 1.27`. The largest FrictionIndex
  averages are now `ShockPropagation 1.88`, `Admm 1.64`, `Sap 1.62`,
  `BGS 1.55`, `Apgd 1.49`, `NNCG 1.48`, and
  `SubspaceMinimization 1.47`.
- The same regenerated profile reports Boxed `RedBlackGaussSeidel 1.95` and
  `ShockPropagation 1.81`, and Standard `InteriorPoint 2.68` with
  `PenalizedFischerBurmeisterNewton` around `2x`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(RedBlackGaussSeidel|Jacobi|Pgs|Tgs|Dantzig|Admm|BGS|Apgd|Sap)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_rbg_jacobi_exact_probe.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
```

Immediate resume guidance:

1. Inspect the current branch with `git status -sb` and
   `git log --oneline --decorate -5`.
2. Finish the current checkpoint by running the focused Python metadata test,
   CSV shape check, required `pixi run lint`, and `git diff --check`.
3. If clean, commit with
   `Extend Jacobi and RedBlackGaussSeidel friction exact paths`.
4. If this checkpoint is already committed, continue with Standard
   `InteriorPoint 2.68`, Standard `PenalizedFischerBurmeisterNewton ~2.00`,
   Boxed `RedBlackGaussSeidel 1.95`, FrictionIndex `ShockPropagation 1.88`,
   or Boxed `ShockPropagation 1.81`.
5. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - LLT-First FrictionIndex Exact Helper

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `e024c24a20a Raise ShockPropagation friction exact gate`.
- Current checkpoint target:
  `Optimize friction exact helper with LLT`.
- Pre-commit resume state: the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 66 commits with the LLT helper
  slice uncommitted.
- After this checkpoint is committed, the branch should become ahead of
  `origin/feature/lcp-solver-interface-demos` by 67 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes, PR creation, and
  other GitHub mutations require explicit maintainer/user approval.

Current implementation slice:

- The shared strict-interior friction-index exact helper now tries LLT first
  for SPD rows and falls back to the previous LU solve when LLT cannot produce
  a validated candidate.
- The helper still uses the existing validation checks before accepting a
  candidate, so active-bound, invalid, or non-SPD rows keep the previous
  fallback behavior.
- The profile CSVs in `docs/background/lcp/figures/`, Python demo summary
  strings, Python metadata assertions, projection-method docs, changelog, and
  this hand-off were refreshed.
- Files changed by this checkpoint:
  `CHANGELOG.md`, `dart/math/lcp/lcp_validation.hpp`,
  `docs/background/lcp/04_projection-methods.md`,
  `docs/background/lcp/figures/performance_profile_boxed.csv`,
  `docs/background/lcp/figures/performance_profile_frictionindex.csv`,
  `docs/background/lcp/figures/performance_profile_standard.csv`,
  `docs/dev_tasks/lcp_solver_interface_demos/README.md`,
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`,
  `python/examples/demos/scenes/lcp_physics.py`, and
  `python/tests/unit/test_py_demo_panels.py`.

Focused and profile evidence:

- Focused probe:
  `build/friction_index_llt_exact_probe.json`.
- Focused 64-contact FrictionIndex exact rows improved from the previous full
  profile roughly as follows:
  `BlockedJacobi 631253.79ns -> 254343.08ns`,
  `BoxedSemiSmoothNewton 430872.80ns -> 248047.48ns`,
  `NNCG 459364.02ns -> 254507.79ns`, and
  `ShockPropagation 466220.22ns -> 270455.13ns`.
- The full regenerated profile reports no FrictionIndex solver average above
  `2x`; the largest FrictionIndex averages are now
  `RedBlackGaussSeidel 1.81`, `Jacobi 1.77`, `Admm 1.65`,
  `ShockPropagation 1.60`, `BGS 1.45`, `Sap 1.44`, and `Apgd 1.41`.
- CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
  columns.
- A `pixi run lint` command completed successfully during the earlier hand-off
  turn. Because this checkpoint edits hand-off docs afterward, rerun the
  required pre-commit lint and diff checks before committing.

Immediate resume guidance:

1. Inspect the current branch with `git status -sb` and
   `git log --oneline --decorate -5`.
2. If this checkpoint is still uncommitted, review the LLT helper slice and
   checkpoint it with `Optimize friction exact helper with LLT`.
3. If this checkpoint is already committed, continue with FrictionIndex
   `RedBlackGaussSeidel 1.81`, FrictionIndex `Jacobi 1.77`, Boxed
   `ShockPropagation 1.66`, Standard `Lemke 1.65`, or FrictionIndex
   `Admm 1.65`.
4. Suggested checkpoint commit title:
   `Optimize friction exact helper with LLT`.
5. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - ShockPropagation Large FrictionIndex Exact Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `58bfb658ed7 Extend Symmetric PSOR exact path to friction rows`.
- Current checkpoint target:
  `Raise ShockPropagation friction exact gate`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 66 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes and other GitHub
  mutations require explicit maintainer/user approval.

Current implementation slice:

- `ShockPropagationSolver::solve()` now allows the shared validated
  strict-interior friction-index exact helper through 192 variables, covering
  the current 64-contact FrictionIndex comparison row.
- The exact path remains limited to non-warm-started, validator-accepted
  strict-interior friction-index rows; larger or active-bound rows continue
  through the layered block path.
- Unit coverage adds a 64-contact exact-path regression for ShockPropagation.
- The profile CSVs in `docs/background/lcp/figures/`, Python demo summary
  strings, Python metadata assertions, other-method docs, changelog, and this
  hand-off were refreshed.

Focused and profile evidence:

- Focused after-run:
  `build/friction_index_shock_gate192_after.json`.
- `BM_LcpCompare/FrictionIndex/ShockPropagation/64` accepted the exact path
  with `iterations=0`, exact residual, and `contract_ok=1.0`.
- The full regenerated profile reports FrictionIndex `ShockPropagation 1.97`.
- Remaining above-`2x` FrictionIndex averages are `BlockedJacobi 2.30` and
  `BoxedSemiSmoothNewton 2.05`.
- CSV shape check showed 200 rows in each checked profile CSV, with 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 23 Standard solver
  columns.

Immediate resume guidance:

1. Inspect the current branch with `git status -sb` and
   `git log --oneline --decorate -5`.
2. If this checkpoint is already committed, continue with FrictionIndex
   `BlockedJacobi 2.30`, then `BoxedSemiSmoothNewton 2.05`.
3. Suggested checkpoint commit title:
   `Raise ShockPropagation friction exact gate`.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Symmetric PSOR FrictionIndex Exact Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `c514269d076 Extend APGD exact paths to boxed and friction rows`.
- Current checkpoint target:
  `Extend Symmetric PSOR exact path to friction rows`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 65 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes and other GitHub
  mutations require explicit maintainer/user approval.

Current implementation slice:

- `SymmetricPsorSolver::solve()` now tries
  `detail::trySolveInteriorFrictionIndexLcp(...)` for non-warm-started medium
  friction-index rows when no custom options are supplied.
- The existing exact-path gate remains conservative:
  `options.customOptions == nullptr`, `!options.warmStart`, and
  `n <= kMaxStrictInteriorFastPathSize`.
- Standard strict-interior exact handling remains on
  `detail::trySolveStrictInteriorStandardLcp(...)`.
- Boxed Symmetric PSOR exact handling was intentionally not kept because the
  focused probe showed slower boxed rows.
- Unit coverage adds `SymmetricPsorSolver` to the friction-index high-overhead
  exact-path smoke test and adds a 16-contact medium friction-index exact-path
  test with `maxIterations = 1`.
- The profile CSVs in `docs/background/lcp/figures/`, Python demo summary
  strings, Python metadata assertions, projection-method docs, changelog, and
  this hand-off were refreshed.

Focused benchmark evidence:

- Accepted focused probe:
  `build/symmetric_psor_findex_exact_probe.json`.
- FrictionIndex `SymmetricPsor` moved approximately:
  - contacts 4: `1873.15ns`, `iterations=3` -> `1158.30ns`,
    `iterations=0`.
  - contacts 16: `16733.51ns`, `iterations=4` -> `13919.53ns`,
    `iterations=0`.
  - contacts 64: `288845.33ns`, `iterations=4` -> `267597.98ns`,
    `iterations=4`; the large row remains iterative.
- Focused build and CTest passed for `BM_LCP_COMPARE`,
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`, and the focused
  validation CTest.

Latest regenerated profile snapshot:

- Standard average ratios: `FischerBurmeisterNewton 1.62`,
  `MinimumMapNewton 1.59`, `PenalizedFischerBurmeisterNewton 1.58`,
  `Apgd 1.57`, `InteriorPoint 1.57`, `SubspaceMinimization 1.55`, with the
  remaining Standard rows below that.
- Boxed average ratios: `ShockPropagation 1.93` was the largest; no Boxed
  solver average was above `2x`.
- FrictionIndex average ratios: `ShockPropagation 2.09`,
  `BoxedSemiSmoothNewton 1.94`, `SubspaceMinimization 1.94`, `NNCG 1.88`,
  `Admm 1.80`, `Staggering 1.80`, `Jacobi 1.76`,
  `BlockedJacobi 1.74`, `SymmetricPsor 1.71`, `Dantzig 1.65`, and
  `Apgd 1.64`.
- CSV shape check had already shown 200 rows in each checked profile CSV, with
  15 Boxed solver columns, 16 FrictionIndex solver columns, and 23 Standard
  solver columns.

Rejected probes captured before the stop instruction:

- `ShockPropagationSolver` friction-index exact gate `48 -> 64` did not take
  the exact path for `ShockPropagation/64` and was reverted.
- `SubspaceMinimizationSolver` friction exact gate `<= 48` regressed
  `SubspaceMinimization/64` to roughly `1.34ms` and was reverted.
- Symmetric PSOR boxed exact handling was slower and was not kept.

Immediate resume guidance:

1. Inspect the current uncommitted state with `git status -sb` and
   `git diff --stat`.
2. Run the required lint and diff checks before committing if they are not
   already recorded in the latest session output.
3. If this checkpoint is already committed, continue with the refreshed
   profile's highest target: FrictionIndex `ShockPropagation 2.09`, then
   inspect `SubspaceMinimization`, `BoxedSemiSmoothNewton`, `NNCG`, `Admm`,
   and `Staggering`.
4. Suggested checkpoint commit title:
   `Extend Symmetric PSOR exact path to friction rows`.
5. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - APGD Boxed/Friction Exact Path

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `19a2250232c Delay ShockPropagation reset after exact path`.
- Current checkpoint target:
  `Extend APGD exact paths to boxed and friction rows`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 64 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes and other GitHub
  mutations require explicit maintainer/user approval.

Current implementation slice:

- `ApgdSolver` now uses the shared validated exact fast path for
  non-warm-started boxed active-set rows and medium friction-index rows when no
  per-call custom APGD options are present.
- The existing 48-variable exact-path size gate is reused, so larger or
  validator-rejected rows stay on the iterative APGD path.
- APGD restart-policy sweep rows still pass custom APGD parameters and therefore
  continue exercising the iterative restart variants.
- Unit coverage now checks boxed and medium friction-index APGD exact-path
  acceptance.
- The checked performance-profile CSVs, Python demo profile summary, Python
  metadata assertions, projection-method background note, changelog, and this
  hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run:
  `build/friction_index_apgd_exact_after.json`.
- Compared with the previous full-profile cache, focused FrictionIndex APGD
  timings were approximately:
  - contacts 4: `1966.2ns -> 1188.1ns`, exact residual.
  - contacts 16: `19687.3ns -> 14199.9ns`, exact residual.
  - contacts 64: `350014.2ns -> 296428.5ns`, still iterative and
    `contract_ok=1.0`.

Final regenerated profile snapshot for this slice:

- Standard average ratios: `FischerBurmeisterNewton 1.54`,
  `InteriorPoint 1.54`, `NNCG 1.54`,
  `PenalizedFischerBurmeisterNewton 1.54`, `MinimumMapNewton 1.53`,
  `Apgd 1.51`, `BoxedSemiSmoothNewton 1.50`, `Lemke 1.50`,
  `SubspaceMinimization 1.47`, `Baraff 1.46`, `BGS 1.44`, `Sap 1.44`,
  `BlockedJacobi 1.43`, `Jacobi 1.40`, `RedBlackGaussSeidel 1.35`,
  `SymmetricPsor 1.34`, `Dantzig 1.33`, `MPRGP 1.33`, `Pgs 1.17`,
  `ShockPropagation 1.15`, `Tgs 1.15`, `Admm 1.14`, and `Direct 1.00`.
- Boxed average ratios: `ShockPropagation 1.74`,
  `BoxedSemiSmoothNewton 1.58`, `Sap 1.56`, `Apgd 1.55`,
  `Admm 1.54`, `BGS 1.54`, `SubspaceMinimization 1.54`,
  `BlockedJacobi 1.50`, `NNCG 1.48`, `Dantzig 1.42`,
  `SymmetricPsor 1.41`, `RedBlackGaussSeidel 1.36`, `Jacobi 1.11`,
  `Tgs 1.04`, and `Pgs 1.00`.
- FrictionIndex average ratios: `ShockPropagation 1.87`,
  `SubspaceMinimization 1.82`, `BoxedSemiSmoothNewton 1.80`,
  `SymmetricPsor 1.79`, `BlockedJacobi 1.77`, `NNCG 1.73`,
  `Dantzig 1.69`, `RedBlackGaussSeidel 1.64`, `Jacobi 1.63`,
  `Staggering 1.62`, `Admm 1.61`, `BGS 1.53`, `Apgd 1.49`,
  `Sap 1.47`, `Pgs 1.07`, and `Tgs 1.00`.
- No refreshed profile surface has a solver average above `2x`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(Apgd|Pgs|Tgs|BGS|Sap)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_apgd_exact_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

Immediate resume guidance:

1. Continue with the highest refreshed FrictionIndex target:
   `ShockPropagation 1.87`.
2. Also inspect `SubspaceMinimization`, `BoxedSemiSmoothNewton`,
   `SymmetricPsor`, `BlockedJacobi`, and `NNCG`, which are the remaining
   near-boundary FrictionIndex rows.
3. Run `pixi run lint` before committing any further slice.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - ShockPropagation Delayed Reset

This is the latest hand-off state. Sections below are historical checkpoints
and may describe their own local "current" state.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `0c6e8bb7902 Tune boxed SSN friction exact gate`.
- Current checkpoint target:
  `Delay ShockPropagation reset after exact path`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 63 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation. Pushes and other GitHub
  mutations require explicit maintainer/user approval.

Current implementation slice:

- `ShockPropagationSolver::solve()` now delays non-warm-start initial-guess
  zeroing until after the validated exact-fast-path attempts and block
  validation.
- The change avoids a zero-vector write when a standard, boxed, or
  friction-index exact candidate is accepted.
- The layered fallback still resets the initial guess before computing fallback
  metrics and layer order, so warm-start and fallback solver behavior stay on
  the existing path.
- The checked performance-profile CSVs, Python demo profile summary, Python
  metadata assertions, background note, changelog, and this hand-off were
  refreshed.

Focused benchmark evidence:

- Focused rows before the experiment, from the committed profile cache, were
  approximately:
  - `BM_LcpCompare/Boxed/ShockPropagation/12`: `1374.0ns`,
    `iterations=0`, `contract_ok=1.0`, `block_count=4`.
  - `BM_LcpCompare/Boxed/ShockPropagation/24`: `7560.7ns`,
    `iterations=0`, `contract_ok=1.0`, `block_count=8`.
  - `BM_LcpCompare/Boxed/ShockPropagation/48`: `16664.5ns`,
    `iterations=0`, `contract_ok=1.0`, `block_count=16`.
- A focused after-run was written to
  `build/boxed_shock_delay_reset_after.json`. The observed ShockPropagation
  timings were approximately:
  - contacts 12: `1376.3ns`, neutral versus baseline.
  - contacts 24: `4303.6ns`, a large focused improvement.
  - contacts 48: `15914.9ns`, a modest focused improvement.

Final regenerated profile snapshot for this slice:

- Standard average ratios: `RedBlackGaussSeidel 1.60`,
  `MinimumMapNewton 1.57`, `Baraff 1.56`, `Lemke 1.56`,
  `PenalizedFischerBurmeisterNewton 1.53`, `InteriorPoint 1.52`,
  `FischerBurmeisterNewton 1.51`, `NNCG 1.51`, `BGS 1.49`,
  `BlockedJacobi 1.46`, `Apgd 1.45`, `SubspaceMinimization 1.45`,
  `SymmetricPsor 1.43`, `BoxedSemiSmoothNewton 1.42`, `Sap 1.42`,
  `MPRGP 1.41`, `Dantzig 1.40`, `Jacobi 1.38`, `Pgs 1.28`,
  `Admm 1.19`, `ShockPropagation 1.19`, `Tgs 1.13`, and `Direct 1.00`.
- Boxed average ratios: `BGS 1.83`, `Sap 1.77`,
  `ShockPropagation 1.75`, `Apgd 1.65`,
  `BoxedSemiSmoothNewton 1.60`, `Admm 1.59`,
  `SubspaceMinimization 1.56`, `NNCG 1.54`, `Dantzig 1.53`,
  `BlockedJacobi 1.51`, `SymmetricPsor 1.47`,
  `RedBlackGaussSeidel 1.44`, `Jacobi 1.15`, `Pgs 1.03`, and `Tgs 1.01`.
- FrictionIndex average ratios: `Apgd 2.08`,
  `ShockPropagation 1.91`, `SubspaceMinimization 1.86`,
  `SymmetricPsor 1.86`, `BoxedSemiSmoothNewton 1.82`,
  `BlockedJacobi 1.81`, `NNCG 1.80`, `RedBlackGaussSeidel 1.78`,
  `Dantzig 1.74`, `Jacobi 1.71`, `Staggering 1.67`,
  `Admm 1.63`, `BGS 1.59`, `Sap 1.54`, `Tgs 1.06`, and `Pgs 1.00`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output /tmp/lcp_profile_check
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

Immediate resume guidance:

1. Continue with the current leading FrictionIndex target: `Apgd` is the only
   refreshed average above `2x`.
2. Also inspect near-boundary FrictionIndex `ShockPropagation`,
   `SymmetricPsor`, `SubspaceMinimization`, `BoxedSemiSmoothNewton`,
   `BlockedJacobi`, and `NNCG`.
3. Run `pixi run lint` before committing any further slice.
4. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Configurable Boxed SSN Friction Exact Gate

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint target:
  `Tune boxed SSN friction exact gate`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 62 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BoxedSemiSmoothNewtonSolver::Parameters` now exposes
  `maxFrictionIndexExactSolveDimension`, defaulting to the previous
  conservative 48-variable strict-interior friction-index exact-solve gate.
- The canonical `BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton` rows raise
  that gate to 192 variables so the 64-contact strict-interior comparison packet
  can use the validated exact solve.
- Other shared benchmark paths keep the default gate unless they explicitly opt
  in, avoiding a failed dense 192x192 solve on active/contact-derived
  friction-index rows.
- dartpy bindings, Python LCP demo metadata, benchmark counters, Newton-method
  background docs, changelog, and tests were updated for the parameter.

Focused benchmark evidence:

- Baseline:
  `build/friction_index_bssn64_probe_baseline.json`.
- After-run:
  `build/friction_index_bssn64_exact_gate_param_after.json`.
- `BoxedSemiSmoothNewton/64`: about `653075.1ns`, `iterations=1`, exact
  residual, no gate counter -> about `447664.5ns`, `iterations=0`, exact
  residual, `boxed_ssn_friction_index_exact_solve_dimension=192`.
- Scope check:
  `build/friction_index_bssn_exact_gate_scope_check.json`.
- Scope check result:
  - `BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton/64` kept
    `exact_gate=192`, `iterations=0`, `contract_ok=1.0`.
  - `BM_LcpBoxedSemiSmoothNewtonLineSearchSweep/FrictionIndex/DefaultSearch`
    kept the default `exact_gate=48`, `iterations=0`, `contract_ok=1.0`.

Final regenerated profile snapshot for this slice:

- FrictionIndex average ratios: `ShockPropagation 1.98`, `Apgd 1.91`,
  `NNCG 1.89`, `SubspaceMinimization 1.85`, `BlockedJacobi 1.79`,
  `Dantzig 1.76`, `BoxedSemiSmoothNewton 1.75`, `Jacobi 1.71`,
  `Staggering 1.71`, `RedBlackGaussSeidel 1.70`, `SymmetricPsor 1.68`,
  `Admm 1.64`, `BGS 1.61`, `Sap 1.46`, `Tgs 1.07`, and `Pgs 1.00`.
- Boxed average ratios: `ShockPropagation 2.10`, `Apgd 1.67`, `Sap 1.66`,
  `BoxedSemiSmoothNewton 1.63`, `SubspaceMinimization 1.59`, `Admm 1.59`,
  `BGS 1.56`, `NNCG 1.54`, `BlockedJacobi 1.54`, `Dantzig 1.47`,
  `RedBlackGaussSeidel 1.40`, `SymmetricPsor 1.38`, `Jacobi 1.13`,
  `Pgs 1.03`, and `Tgs 1.02`.
- Standard average ratios: `Dantzig 1.66`, `Lemke 1.61`, `Baraff 1.55`,
  `MinimumMapNewton 1.51`, `FischerBurmeisterNewton 1.49`, and
  `PenalizedFischerBurmeisterNewton 1.45` are the largest current rows.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers dartpy \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/math/test_lcp.py::test_advanced_boxed_solver_parameters_round_trip_from_dartpy_math \
  python/tests/unit/math/test_lcp.py::test_customized_advanced_boxed_solvers_solve_boxed_problem \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/64' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_bssn64_exact_gate_param_after.json
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton/64|BM_LcpBoxedSemiSmoothNewtonLineSearchSweep/FrictionIndex/DefaultSearch' \
  --benchmark_min_time=0.03s \
  --benchmark_format=json > build/friction_index_bssn_exact_gate_scope_check.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Optimize boxed `ShockPropagation`, now the clearest above-`2x` profile row.
2. Run `pixi run lint` before committing any further slice.
3. Do not push without explicit maintainer/user approval.

## 2026-06-12 Stop-Work Hand-Off

The user explicitly requested stopping implementation work and preserving
handoff context without further verification.

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Last committed checkpoint:
  `19686f271a2 Extend boxed SSN friction exact path to medium rows`.
- Before this hand-off-doc edit, the branch was ahead of
  `origin/feature/lcp-solver-interface-demos` by 60 commits with a clean
  worktree.
- No PR is associated with this branch yet.

Latest post-checkpoint context:

- No solver code changes were accepted after the last committed checkpoint.
- The next target under inspection was large FrictionIndex
  `BoxedSemiSmoothNewton`.
- Focused probe artifact:
  `build/friction_index_bssn64_probe_baseline.json`.
- Probe filter:
  `BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/64`.
- Probe result:
  - `Pgs/64`: about `184735.9ns`, `iterations=8`,
    `complementarity=0.0001289616`, `contract_ok=1.0`.
  - `Tgs/64`: about `175096.2ns`, `iterations=8`,
    `complementarity=0.0001289616`, `contract_ok=1.0`.
  - `BoxedSemiSmoothNewton/64`: about `653075.1ns`, `iterations=1`,
    `complementarity=1.7e-13`, `contract_ok=1.0`,
    `boxed_ssn_pgs_warm_start_iterations=5`.

Resume guidance:

1. First reconcile `git status -sb`, recent commits, and this section.
2. Do not treat the large FrictionIndex `BoxedSemiSmoothNewton` probe as an
   accepted optimization; it is diagnostic evidence only.
3. Avoid changing solver semantics to accept loose PGS/TGS warm starts inside
   `BoxedSemiSmoothNewtonSolver` unless solver tolerances are explicitly
   configured that way.
4. If no clean solver-level win emerges, move to near-boundary FrictionIndex
   `Dantzig`, `ShockPropagation`, or `BlockedJacobi`.

Verification for this hand-off:

- None run. The user explicitly requested no further verification.

## 2026-06-12 Current Continuation - Medium Boxed SSN Friction Exact Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint target:
  `Extend boxed SSN friction exact path to medium rows`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 60 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BoxedSemiSmoothNewtonSolver` now lets the shared validated
  strict-interior friction-index exact shortcut handle non-warm-started packets
  up to 48 variables.
- The 16-contact FrictionIndex comparison packet takes the exact path; larger
  64-contact packets stay on the semi-smooth Newton path because the dense
  shortcut was not a focused benchmark win there.
- Unit coverage now checks that the 48-variable medium friction-index packet
  solves through the zero-iteration exact path.
- The checked performance profile CSVs, Python demo profile summary,
  Newton-methods background note, changelog, Python metadata assertions, and
  this hand-off were refreshed.

Focused benchmark evidence:

- Focused A/B command:
  `BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/`.
- Baseline `build/friction_index_bssn_gate12_baseline.json` used the old
  12-variable exact gate.
- After-run `build/friction_index_bssn_gate48_after.json` used the new
  48-variable exact gate.
- `BoxedSemiSmoothNewton` focused timings:
  - contacts 4: `1309.9ns -> 1268.7ns`
  - contacts 16: `26648.0ns -> 15050.6ns`
  - contacts 64: `696561.3ns -> 682964.4ns`

Final regenerated profile snapshot for this slice:

- FrictionIndex average ratios: `BoxedSemiSmoothNewton 2.55`, `Dantzig 1.98`,
  `ShockPropagation 1.95`, `BlockedJacobi 1.91`, `Apgd 1.89`, `NNCG 1.89`,
  `SymmetricPsor 1.83`, `SubspaceMinimization 1.83`, `Jacobi 1.78`,
  `RedBlackGaussSeidel 1.71`, `Staggering 1.71`, `BGS 1.67`, `Admm 1.62`,
  `Sap 1.47`, `Pgs 1.10`, and `Tgs 1.00`.
- Boxed average ratios: `Sap 1.63`, `Apgd 1.62`, `ShockPropagation 1.60`,
  `BGS 1.57`, `BlockedJacobi 1.56`, `Admm 1.54`,
  `BoxedSemiSmoothNewton 1.53`, `NNCG 1.43`,
  `SubspaceMinimization 1.43`, `SymmetricPsor 1.43`, `Dantzig 1.33`,
  `RedBlackGaussSeidel 1.30`, `Jacobi 1.03`, `Tgs 1.02`, and `Pgs 1.01`.
- Standard average ratios: `Lemke 2.00`, `Baraff 1.99`,
  `InteriorPoint 1.87`, `FischerBurmeisterNewton 1.78`,
  `SubspaceMinimization 1.75`, `NNCG 1.60`, `BGS 1.56`, and
  `Dantzig 1.55` are the largest current rows.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/' \
  --benchmark_min_time=0.1s \
  --benchmark_format=json > build/friction_index_bssn_gate48_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue optimizing the large FrictionIndex `BoxedSemiSmoothNewton` row, or
   inspect near-boundary FrictionIndex `Dantzig`/`ShockPropagation`/
   `BlockedJacobi`.
2. Run `pixi run lint` before committing any further slice.
3. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - ShockPropagation Exact Prevalidation

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint target:
  `Prevalidate ShockPropagation exact shortcuts`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 59 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `ShockPropagationSolver` now attempts its validated exact shortcut before full
  problem validation and layered block setup for non-empty standard, boxed, and
  friction-index problem forms.
- Lightweight custom block/layer structure validation still runs before exact
  acceptance, zero-row problems still validate before zero-row success, and
  non-empty custom friction-index partitions still build/validate blocks before
  the shortcut so invalid custom friction partitions fail as before.
- The checked performance profile CSVs, Python demo profile summary,
  other-methods background note, changelog, and this hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/Boxed/(ShockPropagation|Dantzig|Pgs|Tgs|Jacobi)/`.
- `build/boxed_shock_prevalidate_fast_path_after.json` reported:
  - `ShockPropagation` rows 12: `1407.0ns`
  - `ShockPropagation` rows 24: `4137.6ns`
  - `ShockPropagation` rows 48: `16248.8ns`
- All focused rows reported `contract_ok=1.0`.

Final regenerated profile snapshot for this slice:

- Boxed average ratios: `ShockPropagation 1.84`, `Apgd 1.69`, `Admm 1.64`,
  `BGS 1.64`, `SubspaceMinimization 1.64`, `NNCG 1.58`,
  `BlockedJacobi 1.57`, `BoxedSemiSmoothNewton 1.57`, `Sap 1.54`,
  `RedBlackGaussSeidel 1.49`, `Dantzig 1.44`, `SymmetricPsor 1.36`,
  `Jacobi 1.10`, `Pgs 1.08`, and `Tgs 1.00`.
- FrictionIndex average ratios: `BoxedSemiSmoothNewton 2.37`, `NNCG 2.02`,
  `BlockedJacobi 1.79`, `ShockPropagation 1.75`, `Apgd 1.70`,
  `SubspaceMinimization 1.61`, `Dantzig 1.59`, `Jacobi 1.56`,
  `Staggering 1.53`, `RedBlackGaussSeidel 1.51`, `SymmetricPsor 1.47`,
  `BGS 1.45`, `Admm 1.44`, `Sap 1.40`, `Pgs 1.28`, and `Tgs 1.01`.
- Standard has moderate spread only; no standard row is above `2x`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/(ShockPropagation|Dantzig|Pgs|Tgs|Jacobi)/' \
  --benchmark_min_time=0.03s \
  --benchmark_format=json > build/boxed_shock_prevalidate_fast_path_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue optimizing FrictionIndex `BoxedSemiSmoothNewton` and `NNCG`.
   Boxed `ShockPropagation` is now below `2x` but remains the largest boxed
   moderate row.
2. Run `pixi run lint` before committing any further slice.
3. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - ShockPropagation Empty-Custom Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint target:
  `Skip empty ShockPropagation block prebuild`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 58 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `ShockPropagationSolver` now skips eager friction-index block construction
  when custom options exist but both `blockSizes` and `layers` are empty,
  allowing the validated strict-interior friction-index exact path to run
  before block data construction.
- Non-empty custom block/layer partitions still build and validate block data
  before the exact shortcut, so invalid custom partitions still fail before a
  fast path can accept a solution.
- A broader BGS-delegation experiment was tried and removed because it regressed
  the 64-contact FrictionIndex row.
- The checked performance profile CSVs, Python demo profile summary,
  other-methods background note, changelog, and this hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/FrictionIndex/(ShockPropagation|BGS|Pgs|Tgs)/`.
- `build/friction_index_shock_skip_empty_prebuild_after.json` compared with the
  previous `build/lcp_profile_full.json` cache reported:
  - `ShockPropagation` contacts 4: `2214.6ns -> 1743.3ns`
  - `ShockPropagation` contacts 16: `22983.8ns -> 18426.3ns`
  - `ShockPropagation` contacts 64: `447392.9ns -> 449624.5ns`
- All focused rows reported `contract_ok=1.0`.

Final regenerated profile snapshot for this slice:

- FrictionIndex average ratios: `BoxedSemiSmoothNewton 2.75`, `Apgd 2.04`,
  `ShockPropagation 2.00`, `NNCG 1.93`, `SubspaceMinimization 1.92`,
  `BlockedJacobi 1.83`, `Admm 1.76`, `Jacobi 1.74`, `Staggering 1.74`,
  `BGS 1.73`, `Dantzig 1.73`, `RedBlackGaussSeidel 1.69`,
  `SymmetricPsor 1.66`, `Sap 1.54`, `Tgs 1.12`, and `Pgs 1.00`.
- Boxed `ShockPropagation` is back above `2x` in the refreshed profile
  (`2.65`) and is the next obvious boxed target.
- Standard has moderate spread only; no standard row is above `2x`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(ShockPropagation|BGS|Pgs|Tgs)/' \
  --benchmark_min_time=0.03s \
  --benchmark_format=json > build/friction_index_shock_skip_empty_prebuild_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
git diff --check
```

Immediate next step:

1. Optimize Boxed `ShockPropagation`, then FrictionIndex
   `BoxedSemiSmoothNewton` and `Apgd`.
2. Run `pixi run lint` before committing any further slice.
3. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed SSN Line-Search Early Acceptance

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint target:
  `Accept converged boxed SSN line-search steps`.
- After this checkpoint, the branch should be ahead of
  `origin/feature/lcp-solver-interface-demos` by 57 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BoxedSemiSmoothNewtonSolver` now accepts a line-search step that already
  reaches the natural residual tolerance, avoiding one extra Newton loop whose
  only purpose was to observe convergence at the next iteration header.
- Added unit coverage with a warm-started standard row and `maxIterations=1` so
  the Newton path, not the non-warm-started exact shortcut, proves the early
  line-search acceptance behavior.
- Refreshed checked LCP performance profile CSVs, the Python demo profile
  summary, Newton-method background docs, changelog, and this hand-off.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/`.
- `build/friction_index_bssn_line_search_after.json` compared with
  `build/friction_index_bssn_shock_baseline.json` reported
  `BoxedSemiSmoothNewton` iterations dropping from `2` to `1` on 16- and
  64-contact FrictionIndex packets.
- Focused `BoxedSemiSmoothNewton` absolute times improved from `33270.0ns` to
  `26775.6ns` on 16 contacts and from `829245.2ns` to `754288.4ns` on 64
  contacts. The 4-contact row remains on the zero-iteration exact path.
- All focused rows reported `contract_ok=1.0`.

Final regenerated profile snapshot for this slice:

- FrictionIndex average ratios: `BoxedSemiSmoothNewton 2.44`,
  `ShockPropagation 2.24`, `BlockedJacobi 2.02`, `Apgd 1.89`,
  `RedBlackGaussSeidel 1.89`, `SymmetricPsor 1.79`, `SubspaceMinimization 1.75`,
  `BGS 1.74`, `NNCG 1.73`, `Jacobi 1.72`, `Staggering 1.71`, `Admm 1.63`,
  `Dantzig 1.62`, `Sap 1.37`, `Tgs 1.10`, and `Pgs 1.00`.
- Boxed remains below `2x` for all solvers in the refreshed profile.
- Standard has moderate spread; this slice does not claim Standard completion.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(BoxedSemiSmoothNewton|Pgs|Tgs)/' \
  --benchmark_min_time=0.03s \
  --benchmark_format=json > build/friction_index_bssn_line_search_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
```

Immediate next step:

1. Continue reducing FrictionIndex `ShockPropagation`, `BlockedJacobi`, and
   larger `BoxedSemiSmoothNewton` rows.
2. Run `pixi run lint` before committing any further slice.
3. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior Friction-Index Fast Paths

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path strict-interior friction-index LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 56 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- Added a shared validated strict-interior friction-index exact solve in
  `dart/math/lcp/lcp_validation.hpp`.
- Wired the helper into non-warm-started Dantzig, Blocked Jacobi, BGS, NNCG,
  Subspace Minimization, Staggering, size-gated ShockPropagation, small-packet
  ADMM, and small-packet BoxedSemiSmoothNewton high-level solves.
- Contact-sized local `findex` blocks in Blocked Jacobi, BGS, and
  ShockPropagation try the helper before Dantzig fallback.
- ShockPropagation now validates custom friction block partitions before any
  top-level friction-index shortcut, so invalid custom partitions still fail
  before the fast path can accept a solution.
- SAP remains on its existing regularized Newton path for friction-index rows
  because the dense shortcut was not faster in the refreshed profile.
- Unit coverage now checks the high-overhead strict-interior friction-index
  fast path and preserves the existing invalid custom-block behavior.
- The checked performance profile CSVs, Python demo metadata, projection,
  Newton, Other-methods background docs, changelog, Python metadata assertions,
  and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/FrictionIndex/(Dantzig|BlockedJacobi|BGS|NNCG|SubspaceMinimization|ShockPropagation|Staggering|Admm|BoxedSemiSmoothNewton|Sap)/`.
- `build/friction_index_interior_fast_path_after.json` reported speedups over
  the previous `build/lcp_profile_full.json` cache including:
  - `BlockedJacobi`: `17.71x`, `11.39x`, and `2.80x` for 4, 16, and 64
    contact packets.
  - `BGS`: `9.28x`, `3.39x`, and `1.45x`.
  - `ShockPropagation`: `6.49x`, `3.21x`, and `1.47x`.
  - `Staggering`: `6.13x`, `3.14x`, and `3.14x`.
  - `SubspaceMinimization`: `3.27x`, `2.77x`, and `2.70x`.
  - `NNCG`: `3.68x`, `2.53x`, and `1.66x`.
  - `Dantzig`: `1.90x`, `1.50x`, and `2.04x`.
  - `Admm`: `3.29x` on the 4-contact small-packet exact path; larger rows
    stay on ADMM iteration.
  - `BoxedSemiSmoothNewton`: `2.56x` on the 4-contact small-packet exact path;
    larger rows stay on the semi-smooth Newton path.
  - All focused rows reported `contract_ok=1.0`.

Final regenerated profile snapshot for this slice:

- FrictionIndex: `BlockedJacobi` average ratio improved to `2.00`, BGS to
  `1.69`, ShockPropagation to `2.35`, Staggering to `1.83`,
  SubspaceMinimization to `1.77`, NNCG to `1.83`, Dantzig to `1.83`, and ADMM
  to `1.63`.
- Remaining FrictionIndex rows above `2x`: `BoxedSemiSmoothNewton` (`2.84`) and
  `ShockPropagation` (`2.35`); `BlockedJacobi` is near `2x`.
- Boxed remains below `2x` for all solvers in the refreshed profile.
- Standard remains moderate-spread only; no new standard target became
  dominant.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/FrictionIndex/(Dantzig|BlockedJacobi|BGS|NNCG|SubspaceMinimization|ShockPropagation|Staggering|Admm|BoxedSemiSmoothNewton|Sap)/' \
  --benchmark_min_time=0.02s \
  --benchmark_format=json > build/friction_index_interior_fast_path_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing
   FrictionIndex `BoxedSemiSmoothNewton` and `ShockPropagation`; then revisit
   `BlockedJacobi` large-contact rows if the `2x` boundary remains important.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Remaining Boxed Active-Set Fast Paths

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path remaining boxed active-set LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 55 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BoxedSemiSmoothNewtonSolver`, `SapSolver`, and
  `SubspaceMinimizationSolver` now extend their non-warm-started exact
  fast-path gateways to boxed rows without friction-index coupling.
- `SapSolver` also now uses the shared strict-interior standard-LCP exact solve
  for non-warm-started high-level solves.
- The boxed shortcuts use the shared projected-active-set exact solve and are
  accepted only when the final boxed candidate passes solution validation.
- Warm-started solves, coupled friction-index rows, and validator-rejected rows
  stay on each solver's existing semi-smooth Newton, SAP regularized Newton, or
  PGS-subspace refinement path.
- Unit coverage now checks lower/upper/free boxed BoxedSemiSmoothNewton, SAP,
  and SubspaceMinimization packets that solve in zero iterations, plus SAP's
  standard strict-interior fast path.
- The checked LCP performance profile CSVs, Python demo metadata, projection,
  Newton, and Other-methods background docs, changelog, Python metadata
  assertions, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/Boxed/(BoxedSemiSmoothNewton|Sap|SubspaceMinimization)/`.
- `build/remaining_boxed_projected_active_set_after.json` reported:
  - `SubspaceMinimization` 12 rows: `1316.947ns`
  - `SubspaceMinimization` 24 rows: `4338.628ns`
  - `SubspaceMinimization` 48 rows: `17578.805ns`
  - `Sap` 12 rows: `1359.414ns`
  - `Sap` 24 rows: `4524.363ns`
  - `Sap` 48 rows: `16282.916ns`
  - `BoxedSemiSmoothNewton` 12 rows: `1268.846ns`
  - `BoxedSemiSmoothNewton` 24 rows: `4536.356ns`
  - `BoxedSemiSmoothNewton` 48 rows: `15964.235ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Boxed: `BoxedSemiSmoothNewton` average ratio `1.54`, `Sap` average ratio
  `1.56`, and `SubspaceMinimization` average ratio `1.52`.
- No boxed solver is above `2x` in the refreshed profile.
- Remaining high-ratio targets are now primarily FrictionIndex:
  `BlockedJacobi`, `ShockPropagation`, `BGS`, `Staggering`,
  `SubspaceMinimization`, `NNCG`, `Dantzig`, `BoxedSemiSmoothNewton`, and
  `Admm`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/(BoxedSemiSmoothNewton|Sap|SubspaceMinimization)/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/remaining_boxed_projected_active_set_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing
   FrictionIndex `BlockedJacobi`, `ShockPropagation`, `BGS`, `Staggering`, or
   `SubspaceMinimization` rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed Block Projection Active-Set Fast Paths

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path boxed active-set block projection LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 54 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BlockedJacobiSolver` and `BgsSolver` now extend their non-warm-started
  exact fast-path gateways from strict-interior standard rows to boxed rows
  without friction-index coupling.
- The boxed shortcuts use the shared projected-active-set exact solve and are
  accepted only when the final boxed candidate passes solution validation.
- Explicit custom block options still build and validate block structure before
  any exact shortcut is accepted.
- Warm-started solves and friction-index rows stay on the existing block
  iteration paths.
- Unit coverage now checks lower/upper/free boxed BlockedJacobi and BGS
  packets that solve in zero iterations.
- The checked LCP performance profile CSVs, Python demo metadata, projection
  background docs, changelog, Python metadata assertions, and this dev-task
  hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Boxed/(BlockedJacobi|BGS)/`.
- `build/block_projection_boxed_projected_active_set_after.json` reported:
  - `BlockedJacobi` 12 rows: `1764.970ns`
  - `BlockedJacobi` 24 rows: `4257.723ns`
  - `BlockedJacobi` 48 rows: `18685.986ns`
  - `BGS` 12 rows: `1621.035ns`
  - `BGS` 24 rows: `6811.200ns`
  - `BGS` 48 rows: `17464.305ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Boxed: `BlockedJacobi` average ratio `1.58` and `BGS` average ratio `1.66`,
  no longer high-ratio boxed targets.
- Current Boxed high-ratio targets are `BoxedSemiSmoothNewton`, `Sap`, and
  `SubspaceMinimization`.
- FrictionIndex high-ratio targets remain `BlockedJacobi`, `BGS`,
  `ShockPropagation`, `Staggering`, `SubspaceMinimization`, `NNCG`,
  `BoxedSemiSmoothNewton`, `Dantzig`, and `Admm`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/(BlockedJacobi|BGS)/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/block_projection_boxed_projected_active_set_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing Boxed
   `BoxedSemiSmoothNewton`/`Sap` or `SubspaceMinimization`, or switching to the
   larger FrictionIndex high-ratio rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed NNCG Projected Active-Set Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path boxed active-set NNCG LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 53 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `NncgSolver` now extends its non-warm-started exact fast-path gateway from
  strict-interior standard rows to boxed rows without friction-index coupling.
- The boxed shortcut uses the shared projected-active-set exact solve and is
  accepted only when the final boxed candidate passes solution validation.
- Warm-started solves and friction-index rows stay on the existing
  PGS-preconditioned NNCG iteration path.
- Unit coverage now checks a lower/upper/free boxed NNCG packet that solves in
  zero iterations.
- The checked LCP performance profile CSVs, Python demo metadata, projection
  background docs, changelog, Python metadata assertions, and this dev-task
  hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Boxed/NNCG/`.
- `build/nncg_boxed_projected_active_set_after.json` reported:
  - 12 rows: `1184.486ns`
  - 24 rows: `3822.184ns`
  - 48 rows: `13505.529ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Boxed: `NNCG` average ratio `1.68`, no longer a high-ratio boxed target.
- Current Boxed high-ratio targets are `BlockedJacobi`, `BGS`,
  `BoxedSemiSmoothNewton`, `Sap`, and `SubspaceMinimization`.
- FrictionIndex high-ratio targets remain `BlockedJacobi`, `ShockPropagation`,
  `Staggering`, `BGS`, `SubspaceMinimization`, `NNCG`,
  `BoxedSemiSmoothNewton`, `Dantzig`, and `Admm`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/NNCG/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/nncg_boxed_projected_active_set_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing
   Boxed `BlockedJacobi`/`BGS`, `BoxedSemiSmoothNewton`/`Sap`, or
   `SubspaceMinimization`, or switching to the larger FrictionIndex high-ratio
   rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed Dantzig Projected Active-Set Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path boxed active-set Dantzig LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 52 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `DantzigSolver` now extends its non-warm-started `LcpProblem` exact fast path
  from strict-interior standard rows to boxed rows without friction-index
  coupling.
- The boxed shortcut uses the shared projected-active-set exact solve and is
  accepted only when the final boxed candidate passes solution validation.
- Warm-started solves and low-level matrix/scratch calls stay on the
  ODE-derived pivoting path; friction-index rows continue to use the existing
  Dantzig pivot/refinement route.
- Unit coverage now checks a lower/upper/free boxed Dantzig packet that solves
  in zero iterations.
- The checked LCP performance profile CSVs, Python demo metadata, pivoting
  background docs, changelog, Python metadata assertions, and this dev-task
  hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Boxed/Dantzig/`.
- `build/dantzig_boxed_projected_active_set_after.json` reported:
  - 12 rows: `1281.450ns`
  - 24 rows: `3993.311ns`
  - 48 rows: `14533.545ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Boxed: `Dantzig` average ratio `1.46`, no longer a high-ratio boxed target.
- Current Boxed high-ratio targets are `NNCG`, `BlockedJacobi`, `BGS`,
  `BoxedSemiSmoothNewton`, `Sap`, and `SubspaceMinimization`.
- FrictionIndex high-ratio targets remain `BlockedJacobi`, `BGS`,
  `ShockPropagation`, `Staggering`, `SubspaceMinimization`, `NNCG`, `Dantzig`,
  `BoxedSemiSmoothNewton`, and `Admm`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/Dantzig/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/dantzig_boxed_projected_active_set_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing
   Boxed `NNCG`, `BlockedJacobi`/`BGS`, or `BoxedSemiSmoothNewton`/`Sap`, or
   switching to the larger FrictionIndex high-ratio rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed ShockPropagation Projected Active-Set Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path boxed active-set ShockPropagation LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 51 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `ShockPropagationSolver` now extends its exact fast-path gateway from
  strict-interior standard rows to boxed LCP rows without friction-index
  coupling.
- The boxed shortcut runs after lightweight custom block/layer structure
  validation, before block/layer data construction, and accepts only candidates
  that pass the shared projected-active-set boxed validator.
- Warm-started, friction-index, and validator-rejected boxed rows stay on the
  existing block/layer sweep path.
- Unit coverage now checks both the zero-iteration boxed fast path and invalid
  boxed custom block-size rejection before any fast-path bypass.
- The checked LCP performance profile CSVs, Python demo metadata, Other-methods
  background docs, changelog, Python metadata assertions, and this dev-task
  hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Boxed/ShockPropagation/`.
- `build/shockprop_boxed_projected_active_set_after.json` reported:
  - 12 rows: `1316.398ns`
  - 24 rows: `4229.786ns`
  - 48 rows: `13883.343ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Boxed: `ShockPropagation` average ratio `1.81`, down from the dominant
  high-ratio boxed target class.
- Current Boxed high-ratio targets are `Dantzig`, `NNCG`, `BlockedJacobi`,
  `BGS`, `BoxedSemiSmoothNewton`, `Sap`, and `SubspaceMinimization`.
- FrictionIndex high-ratio targets remain `BlockedJacobi`, `Staggering`,
  `ShockPropagation`, `BGS`, `SubspaceMinimization`, `NNCG`, `Dantzig`,
  `BoxedSemiSmoothNewton`, and `Admm`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/ShockPropagation/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/shockprop_boxed_projected_active_set_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing
   Boxed `Dantzig`, `NNCG`, `BlockedJacobi`/`BGS`, or
   `BoxedSemiSmoothNewton`/`Sap`, or switching to the larger FrictionIndex
   high-ratio rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed ADMM Projected Active-Set Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path boxed active-set ADMM LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 50 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- Added a shared validated projected-active-set boxed-LCP exact-solve helper.
  It uses the unconstrained solve only to propose lower/upper/free rows, solves
  the free block exactly with active rows fixed, and accepts the shortcut only
  when the final boxed solution passes the existing validator.
- `AdmmSolver` now uses that helper for default, non-warm-started boxed LCPs
  without friction-index coupling before allocating ADMM iteration workspace.
- Warm-started, explicit custom-option, standard strict-interior, and
  friction-index ADMM paths retain their existing behavior.
- Unit coverage now includes an active lower/upper/free boxed ADMM packet that
  solves in zero iterations.
- The LCP performance profile CSVs, Python demo metadata, ADMM background docs,
  changelog, Python metadata assertions, and this dev-task hand-off were
  refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Boxed/Admm/`.
- `build/admm_boxed_projected_active_set_after.json` reported:
  - 12 rows: `1217.118ns`
  - 24 rows: `3806.814ns`
  - 48 rows: `15540.370ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Boxed: `Admm` average ratio `1.56`, no longer a high-ratio boxed target.
- Standard: `Admm` average ratio `1.19`.
- Remaining high-ratio targets are Boxed `ShockPropagation`, `NNCG`,
  `Dantzig`, `BlockedJacobi`, `BoxedSemiSmoothNewton`, `BGS`,
  `SubspaceMinimization`, and `Sap`; FrictionIndex `BlockedJacobi`,
  `ShockPropagation`, `Staggering`, `BGS`, `SubspaceMinimization`, `NNCG`,
  `Dantzig`, `BoxedSemiSmoothNewton`, `Admm`, and `Apgd`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers BM_LCP_COMPARE \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Boxed/Admm/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/admm_boxed_projected_active_set_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing
   Boxed `ShockPropagation`, `NNCG`, `Dantzig`, or the boxed block/Newton rows,
   or switching to the larger FrictionIndex high-ratio rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior ShockPropagation Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path strict-interior ShockPropagation LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 49 commits.
- No PR is associated with this branch yet.
- No push has been performed for this continuation.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `ShockPropagationSolver` now moves the strict-interior standard-LCP fast path
  ahead of block/layer matrix construction after lightweight custom block/layer
  structure validation.
- The shortcut prefers an `Eigen::LLT` exact solve for SPD rows and falls back
  to the shared strict-interior linear-solve helper if the LLT candidate does
  not validate.
- Invalid custom block sizes and layers still return `InvalidProblem` before a
  strict-interior exact solve is accepted.
- Active-bound, non-standard, warm-started, boxed, and friction-index
  ShockPropagation paths stay on the existing iterative/block paths.
- `ShockPropagationSolverCoverage.SolvesWithCustomLayers` now expects the
  custom-layer strict-interior standard problem to solve in zero iterations.
- The checked LCP performance profile CSVs, Python demo metadata, Other-methods
  background docs, changelog, Python metadata test, and this dev-task hand-off
  were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Standard/ShockPropagation/`.
- `build/shockprop_strict_interior_after.json` reported:
  - 12 rows: `825.964ns`
  - 24 rows: `2188.264ns`
  - 48 rows: `9848.529ns`
  - 96 rows: `43487.181ns`
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Standard: `ShockPropagation` average ratio `1.21`.
- Current Standard follow-up targets are moderate iterative rows such as
  `Apgd`, `FischerBurmeisterNewton`, `MinimumMapNewton`, `InteriorPoint`,
  `Lemke`, `MPRGP`, and `SubspaceMinimization`.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `NNCG`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, `NNCG`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/ShockPropagation/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/shockprop_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run build
DART_PARALLEL_JOBS="$JOBS" CTEST_PARALLEL_LEVEL="$JOBS" \
  CMAKE_BUILD_PARALLEL_LEVEL="$JOBS" pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile. The next natural work is reducing the
   remaining moderate Standard rows or switching to the larger Boxed/Friction
   high-ratio rows.
2. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior Projection Iterator Fast Paths

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint:
  `Fast path strict-interior projection iterator LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 48 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `ApgdSolver`, `JacobiSolver`, `SymmetricPsorSolver`, and
  `RedBlackGaussSeidelSolver` now try the shared validated strict-interior
  standard-LCP fast path for non-warm-started solves without explicit
  per-solve custom options.
- Each shortcut has a profile-shaped size guard so larger packets stay on the
  existing iterative projection path when the dense linear solve is not
  profitable.
- The strict-interior projection/block unit coverage now includes APGD, Jacobi,
  Symmetric PSOR, and Red-Black Gauss-Seidel.
- The checked LCP performance profile CSVs, Python demo metadata, projection
  background docs, changelog, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/Standard/(Apgd|Jacobi|SymmetricPsor|RedBlackGaussSeidel)/`.
- Compared to the previous full profile cache:
  - `Apgd`: mean `0.745`; best `0.615`; worst `0.988`.
  - `Jacobi`: mean `0.825`; best `0.490`; worst `1.101`.
  - `RedBlackGaussSeidel`: mean `0.752`; best `0.413`; worst `1.024`.
  - `SymmetricPsor`: mean `0.683`; best `0.476`; worst `0.860`.
  - All focused rows reported `contract_ok=1.0`; fast-pathed rows reported
    `iterations=0`.

Final regenerated profile snapshot for this slice:

- Standard averages: `Apgd` `1.46`, `Jacobi` `1.44`,
  `RedBlackGaussSeidel` `1.74`, and `SymmetricPsor` `1.56`.
- Current Standard follow-up targets are `ShockPropagation`,
  `RedBlackGaussSeidel`, and moderate `FischerBurmeisterNewton`, `NNCG`,
  `Lemke`, `SymmetricPsor`, `InteriorPoint`, and similar mid-pack rows.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, `Nncg`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/(Apgd|Jacobi|SymmetricPsor|RedBlackGaussSeidel)/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/projection_iterators_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
```

Immediate next step:

1. Continue from the refreshed profile. The next natural Standard target is
   `ShockPropagation`, or switch to the Boxed/FrictionIndex high-ratio rows.
   Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior ADMM Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint: `Fast path strict-interior ADMM LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 47 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- Non-warm-started, default-option `AdmmSolver` calls now try a validated
  strict-interior standard-LCP exact-solve path before allocating ADMM
  iteration workspace.
- The fast path prefers an LLT solve for SPD rows and falls back to the shared
  strict-interior linear-solve helper if the LLT candidate does not validate.
- Boxed, friction-index, warm-started, and explicit custom-option ADMM calls
  stay on the existing operator-splitting loop.
- The strict-interior "Other" unit coverage now includes `AdmmSolver` with
  `warmStart=false`.
- The checked LCP performance profile CSVs, Python demo metadata, ADMM
  background docs, changelog, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Standard/Admm/`.
- Compared to the previous full profile cache:
  - 12 rows: `0.216`
  - 24 rows: `0.367`
  - 48 rows: `0.597`
  - 96 rows: `0.744`
  - Mean focused ratio `0.481`; best `0.216`; worst `0.744`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Standard: `Admm` average ratio `1.07`, with 2 wins across 4 solved Standard
  profile rows.
- Current Standard follow-up targets are moderate iterative rows:
  `Apgd`, `Jacobi`, `SymmetricPsor`, `RedBlackGaussSeidel`,
  `ShockPropagation`, and `Sap`.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, `Nncg`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/Admm/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/admm_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
```

Immediate next step:

1. Continue from the refreshed profile and target one of the remaining
   moderate Standard rows, or start reducing the Boxed/FrictionIndex high-ratio
   rows. Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior MPRGP Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint: `Fast path strict-interior MPRGP LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 46 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- Default `MprgpSolver` calls now try the shared validated strict-interior
  standard-LCP fast path after MPRGP's symmetry and positive-definite checks.
- The fast path reuses the existing LLT factorization instead of factoring the
  matrix again, so it preserves MPRGP's SPD contract while removing the
  reduced-gradient loop on strict-interior rows.
- Custom-option and warm-started MPRGP calls stay on the iterative path so
  parameter stress tests and user tuning remain observable.
- The strict-interior "Other" unit coverage now includes `MprgpSolver`.
- The checked LCP performance profile CSVs, Python demo metadata, background
  docs, changelog, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Standard/MPRGP/`.
- Compared to the previous full profile cache:
  - 12 rows: `0.390`
  - 24 rows: `0.481`
  - 48 rows: `0.580`
  - 96 rows: `0.458`
  - Mean focused ratio `0.477`; best `0.390`; worst `0.580`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Standard: `MPRGP` average ratio `1.70`; it is no longer a Standard profile
  high-ratio target.
- Current Standard high-ratio target is `Admm`, with moderate follow-up rows
  including `Apgd`, `Jacobi`, `SymmetricPsor`, `RedBlackGaussSeidel`, and
  `ShockPropagation`.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `Staggering`, `ShockPropagation`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel 1
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/MPRGP/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/mprgp_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run build
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile and target `Admm` on Standard rows, or
   one of the remaining boxed/friction-index high-ratio rows. Do not push
   without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior Dantzig Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint: `Fast path strict-interior Dantzig LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 45 commits.
- No PR is associated with this branch yet.
- Pushes still require explicit maintainer/user approval.

Current implementation slice:

- `DantzigSolver` now tries the shared validated strict-interior standard-LCP
  fast path through the `LcpProblem` solver interface before allocating the
  ODE-derived pivot workspace.
- The fast path runs after problem validation and only when callers are not
  warm-starting.
- Boxed, friction-index, warm-started, and low-level matrix/scratch Dantzig
  calls stay on the existing pivoting path.
- The strict-interior pivot/barrier unit coverage now includes `DantzigSolver`
  with `warmStart=false`.
- The checked LCP performance profile CSVs, Python demo metadata, background
  docs, changelog, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Standard/Dantzig/`.
- Current focused rerun reported `contract_ok=1.0` and `iterations=0` for
  12-, 24-, 48-, and 96-row Standard Dantzig packets.

Final regenerated profile snapshot for this slice:

- Standard: `Dantzig` average ratio `1.24`; it is no longer a Standard profile
  laggard.
- Current Standard high-ratio targets are `MPRGP`, `Admm`, plus moderate
  `Jacobi`, `RedBlackGaussSeidel`, `Apgd`, and `SymmetricPsor` rows.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `Staggering`, `ShockPropagation`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel 1
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/Dantzig/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/dantzig_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run build
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile and target `Admm` or `MPRGP` on
   Standard rows, or one of the remaining boxed/friction-index high-ratio rows.
   Do not push without explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior Boxed Semi-Smooth Newton Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Top local checkpoint: `Fast path strict-interior boxed Newton LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 44 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BoxedSemiSmoothNewtonSolver` now tries the shared validated
  strict-interior standard-LCP fast path after parameter validation when the
  caller is not warm-starting.
- Warm-started boxed and friction-index rows stay on the existing
  residual-reducing PGS warm-start and semi-smooth Newton line-search path.
- The strict-interior Newton unit coverage now includes
  `BoxedSemiSmoothNewtonSolver` with `warmStart=false`.
- The checked LCP performance profile CSVs, Python demo metadata, background
  docs, changelog, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Standard/BoxedSemiSmoothNewton/`.
- Compared to the previous full profile cache:
  - 12 rows: `0.365`
  - 24 rows: `0.384`
  - 48 rows: `0.731`
  - 96 rows: `0.416`
  - Mean focused ratio `0.474`; best `0.365`; worst `0.731`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Standard: `BoxedSemiSmoothNewton` average ratio `1.28`; it is no longer a
  Standard profile laggard.
- Current Standard high-ratio targets are `Admm`, `Dantzig`, `MPRGP`, plus
  moderate `RedBlackGaussSeidel`, `Jacobi`, `Apgd`, and `SymmetricPsor` rows.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `Staggering`, `ShockPropagation`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel 1
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/BoxedSemiSmoothNewton/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/boxed_ssn_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run build
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile and target the remaining boxed and
   friction-index high-ratio rows. Do not push without explicit maintainer/user
   approval.

## 2026-06-12 Current Continuation - Strict-Interior BGS Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current `HEAD`: `f97ce3b9bf6 Fast path strict-interior projection LCPs`.
- This section records the local checkpoint titled
  `Fast path strict-interior BGS LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 43 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current implementation slice:

- `BgsSolver` now tries the shared validated strict-interior standard-LCP fast
  path for small and medium standard rows when the caller is not warm-starting.
- The fast path is size-capped at 48 rows because focused evidence showed the
  dense linear solve regressed the 96-row BGS profile row relative to the
  existing scalar block sweep.
- Default-option solves can take the fast path before block construction, while
  explicit custom block partitions are still validated before the fast path is
  accepted.
- The strict-interior projection/block unit coverage now includes `BgsSolver`.
- The checked LCP performance profile CSVs, Python demo metadata, background
  docs, changelog, and this dev-task hand-off were refreshed.

Focused benchmark evidence:

- Focused after-run: `BM_LcpCompare/Standard/BGS/`.
- Compared to the previous full profile cache:
  - 12 rows: `0.257`
  - 24 rows: `0.341`
  - 48 rows: `0.493`
  - 96 rows: `0.827`
  - Mean focused ratio `0.480`; best `0.257`; worst `0.827`.
  - All focused rows reported `contract_ok=1.0`; the fast-path rows reported
    `iterations=0`, while the 96-row capped fallback reported `iterations=5`.

Final regenerated profile snapshot for this slice:

- Standard: `BGS` average ratio `1.39`; it is no longer a Standard profile
  laggard.
- Current Standard high-ratio targets are `BoxedSemiSmoothNewton`, `Admm`,
  `Dantzig`, `MPRGP`, plus moderate `Jacobi`, `Apgd`, and `SymmetricPsor`
  rows.
- Boxed/FrictionIndex targets remain unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`, and `BlockedJacobi`; FrictionIndex
  `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel 1
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/BGS/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/bgs_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run build
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile and target the remaining boxed and
   friction-index high-ratio rows. Do not push without explicit maintainer/user
   approval.

## 2026-06-12 Current Continuation - Strict-Interior Projection/Block Fast Path

Current branch state:

- Branch: `feature/lcp-solver-interface-demos`.
- Current `HEAD`: `2d74356e830 Fast path strict-interior Newton LCPs`.
- This section records the local checkpoint titled
  `Fast path strict-interior projection LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 42 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current implementation slice:

- `NncgSolver` and `SubspaceMinimizationSolver` now use
  `detail::trySolveStrictInteriorStandardLcp()` after parameter validation and
  before PGS setup when there is no warm start.
- `BlockedJacobiSolver` and `ShockPropagationSolver` now use local helpers that
  let default-option strict-interior standard rows skip block/layer setup, while
  preserving custom block/layer validation before the fast path.
- Added unit coverage:
  `StandardStrictInteriorFastPath.ProjectionAndBlockSolversUseLinearSolve`.
- Adjusted the existing Blocked Jacobi max-iteration coverage to use a boundary
  target so it still exercises the iterative path.
- Regenerated the checked LCP performance profile CSVs and refreshed
  demo/docs/changelog text.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/Standard/(BlockedJacobi|NNCG|ShockPropagation|SubspaceMinimization)/`.
- Compared to the previous full profile cache:
  - `BlockedJacobi`: `0.1646`, `0.1742`, `0.4021`, `0.5214`
  - `NNCG`: `0.1483`, `0.2532`, `0.3607`, `0.5038`
  - `SubspaceMinimization`: `0.3274`, `0.3717`, `0.3653`, `0.3545`
  - `ShockPropagation`: `0.2511`, `0.3449`, `0.5894`, `0.8568`
  - Mean focused ratio `0.3743`; best `0.1483`; worst `0.8568`.
  - All focused rows reported `contract_ok=1.0` and `iterations=0`.

Final regenerated profile snapshot for this slice:

- Standard: `BlockedJacobi` average ratio `1.26`, `NNCG` `1.48`,
  `SubspaceMinimization` `1.32`, and `ShockPropagation` `1.68`; these are no
  longer Standard profile laggards.
- Current Standard high-ratio targets are `BGS`, `Admm`,
  `BoxedSemiSmoothNewton`, `Dantzig`, and `MPRGP`.
- Boxed/FrictionIndex targets are unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`; FrictionIndex `BlockedJacobi`, `BGS`,
  `ShockPropagation`, `Staggering`, and `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel 1
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j 1
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/(BlockedJacobi|NNCG|ShockPropagation|SubspaceMinimization)/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/projection_block_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run build
DART_PARALLEL_JOBS=1 CTEST_PARALLEL_LEVEL=1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
  pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile and target the remaining boxed and
   friction-index high-ratio rows. Do not push without explicit maintainer/user
   approval.

## 2026-06-12 Current Continuation - Strict-Interior Newton Fast Path

Current branch state for this slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Parent checkpoint:
  `9057b15a9bf Fast path strict-interior standard LCPs`.
- Latest local checkpoint: `Fast path strict-interior Newton LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 41 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current implementation slice:

- Reused `detail::trySolveStrictInteriorStandardLcp()` in
  `MinimumMapNewtonSolver`, `FischerBurmeisterNewtonSolver`, and
  `PenalizedFischerBurmeisterNewtonSolver`.
- The fast path runs only after solver-parameter validation and only when the
  caller is not warm-starting, so invalid custom options and warm-started
  Newton behavior stay on the existing paths.
- Added unit coverage proving all three Newton solvers report zero-iteration
  success on a validated strict-interior standard LCP.
- Regenerated the checked LCP performance profile CSV artifacts and refreshed
  demo/docs/changelog text.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/Standard/(FischerBurmeisterNewton|PenalizedFischerBurmeisterNewton|MinimumMapNewton)/`.
- Compared to the prior full profile cache:
  - Minimum Map Newton row runtime ratios: `0.193`, `0.264`, `0.310`,
    `0.114`.
  - Fischer-Burmeister Newton row runtime ratios: `0.099`, `0.119`,
    `0.153`, `0.135`.
  - Penalized Fischer-Burmeister Newton row runtime ratios: `0.097`,
    `0.073`, `0.163`, `0.139`.
  - Mean focused ratio was `0.155`; best `0.073`; worst `0.310`.
  - All focused after rows reported `contract_ok=1.0` and `iterations=0`.
- Focused validation CTest
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` passed.

Final regenerated profile snapshot for this slice:

- Standard: `MinimumMapNewton` average ratio `1.43`,
  `PenalizedFischerBurmeisterNewton` `1.55`, and
  `FischerBurmeisterNewton` `1.57`; these are no longer Standard profile
  laggards.
- Current Standard high-ratio targets are `BlockedJacobi`, `Nncg`,
  `ShockPropagation`, and `SubspaceMinimization`.
- Boxed/FrictionIndex targets are unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`; FrictionIndex `BlockedJacobi`, `BGS`,
  `ShockPropagation`, `Staggering`, and `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/(FischerBurmeisterNewton|PenalizedFischerBurmeisterNewton|MinimumMapNewton)/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/newton_strict_interior_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile; do not push without explicit
   maintainer/user approval.

## 2026-06-12 Current Continuation - Strict-Interior Standard Fast Path

Checkpoint hand-off:

- The strict-interior standard-LCP fast path has been implemented and verified
  locally.
- This section records the checkpoint titled
  `Fast path strict-interior standard LCPs`.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current branch state for this slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Parent checkpoint: `4e6e57e7ccb Reuse ADMM iteration workspace`.
- Latest local checkpoint: `Fast path strict-interior standard LCPs`.
- After this checkpoint, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 40 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation after the ADMM checkpoint;
  pushes still require explicit maintainer/user approval.

Current implementation slice:

- Added a shared `detail::trySolveStrictInteriorStandardLcp()` helper.
- `LemkeSolver`, `BaraffSolver`, and `InteriorPointSolver` use it when the
  caller is not warm-starting.
- The fast path solves the unconstrained standard linear system and accepts the
  result only when the candidate is strictly positive and passes the standard
  LCP validator. Boundary, active-set, boxed, and friction-index cases still run
  the existing pivot/path-following or fallback paths.
- Added unit coverage proving Lemke, Baraff, and Interior Point report
  zero-iteration success on a validated strict-interior standard LCP.
- Regenerated the checked LCP performance profile CSV artifacts and refreshed
  demo/docs/changelog text.

Checkpoint surface:

- Core implementation: `dart/math/lcp/lcp_validation.hpp`,
  `dart/math/lcp/pivoting/lemke_solver.cpp`,
  `dart/math/lcp/pivoting/baraff_solver.cpp`, and
  `dart/math/lcp/other/interior_point_solver.cpp`.
- Coverage and user-facing metadata:
  `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/examples/demos/scenes/lcp_physics.py`, and `CHANGELOG.md`.
- Background/profile artifacts:
  `docs/background/lcp/03_pivoting-methods.md`,
  `docs/background/lcp/06_other-methods.md`, and
  `docs/background/lcp/figures/performance_profile_*.csv`.
- Hand-off docs: this file and
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.

Focused benchmark evidence:

- Focused after-run:
  `BM_LcpCompare/Standard/(Lemke|Baraff|InteriorPoint)/`.
- Compared to the prior full profile cache:
  - Lemke row runtime ratios: `0.058`, `0.033`, `0.018`, `0.004`.
  - Baraff row runtime ratios: `0.083`, `0.052`, `0.026`, `0.013`.
  - Interior Point row runtime ratios: `0.021`, `0.021`, `0.028`, `0.013`.
  - Mean focused ratio was `0.031`; best `0.004`; worst `0.083`.
  - All focused after rows reported `contract_ok=1.0` and `iterations=0`.
- Focused validation CTest
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` passed.

Final regenerated profile snapshot for this slice:

- Standard: `Lemke` average ratio `1.42`, `Baraff` `1.35`, and
  `InteriorPoint` `1.44`; these are no longer Standard profile laggards.
- Current Standard high-ratio targets are
  `PenalizedFischerBurmeisterNewton`, `FischerBurmeisterNewton`,
  `MinimumMapNewton`, `Nncg`, `BlockedJacobi`, and `ShockPropagation`.
- Boxed/FrictionIndex targets are unchanged in kind: Boxed `Admm`,
  `ShockPropagation`, `Dantzig`, `Nncg`; FrictionIndex `BlockedJacobi`, `BGS`,
  `ShockPropagation`, `Staggering`, and `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_additional_solvers \
           UNIT_math_lcp_math_lcp_all_solvers_smoke \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(additional_solvers|all_solvers_smoke|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/Standard/(Lemke|Baraff|InteriorPoint)/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/strict_interior_standard_after.json
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j "$JOBS"
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Immediate next step:

1. On resume, inspect `git status -sb`, `git log -5 --oneline --decorate`, and
   `git diff --stat` before acting.
2. Continue from the refreshed profile; do not push without explicit
   maintainer/user approval.

## 2026-06-12 Current Continuation - ADMM Workspace Reuse

Current branch state for this slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Latest local checkpoint: `Reuse ADMM iteration workspace`.
- Parent checkpoint:
  `e8e569ca5af Warm start boxed semi-smooth Newton profiles`.
- At this checkpoint and before pushing, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 39 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation; pushes still require
  explicit maintainer/user approval.

Current implementation slice:

- `AdmmSolver` now reuses its right-hand-side and projected-step vectors across
  ADMM iterations.
- The unused per-iteration `xPrev` copy was removed.
- The operator-splitting updates, adaptive-rho behavior, projection behavior,
  convergence tests, and public parameter surface are unchanged.
- The checked LCP performance profile CSV artifacts were regenerated.
- The Python demo profile summary, panel metadata test, ADMM background docs,
  and `CHANGELOG.md` were updated for the refreshed profile and workspace
  reuse note.

Focused ADMM evidence:

- Focused after-run:
  `BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Admm/`.
- Compared to the prior full profile cache:
  - Standard row runtime ratios: `0.643`, `0.757`, `0.789`, `0.691`.
  - Boxed row runtime ratios: `0.688`, `0.626`, `0.679`.
  - FrictionIndex row runtime ratios: `0.492`, `0.532`, `0.486`.
  - Mean focused ratio was `0.638`; best `0.486`; worst `0.789`.
  - All focused after rows reported `contract_ok=1.0`.
- The focused validation CTest
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` passed.

Final regenerated profile snapshot for this slice:

- Standard: `Admm` average ratio is now `2.61`.
- Boxed: `Admm` average ratio is now `14.90`, with
  `ShockPropagation`, `Dantzig`, and `Nncg` also current high-ratio targets.
- FrictionIndex: `Admm` average ratio is now `2.55`; current high-ratio
  targets remain `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Admm/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/admm_workspace_after.json
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_lcp_validation_and_solvers$' \
  -j "$JOBS"
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Immediate next step:

1. Continue from the refreshed profile. The clearest next targets are Standard
   `Lemke`, `InteriorPoint`, and `Baraff`; Boxed `Admm`,
   `ShockPropagation`, `Dantzig`, and `Nncg`; and FrictionIndex
   `BlockedJacobi`, `BGS`, `ShockPropagation`, `Staggering`, and
   `SubspaceMinimization`.
2. Push only after explicit maintainer/user approval.

## 2026-06-12 Current Continuation - Boxed SSN Warm-Start Verified

Current branch state for this checkpoint:

- Branch: `feature/lcp-solver-interface-demos`.
- Latest local checkpoint: `Warm start boxed semi-smooth Newton profiles`.
- The parent checkpoint is
  `ec2d642c945 Update LCP solver handoff after SAP checkpoint`.
- At this checkpoint and before pushing, the branch is ahead of
  `origin/feature/lcp-solver-interface-demos` by 38 commits.
- No PR is associated with this branch yet.
- No push has been performed in this continuation; pushes still require
  explicit maintainer/user approval.

Current implementation slice:

- Added optional `BoxedSemiSmoothNewtonSolver::Parameters` fields:
  `maxPgsWarmStartIterations` and `pgsWarmStartRelaxation`.
- The solver now optionally runs a short `PgsSolver` warm start before Newton
  iterations and accepts the candidate only when it is finite and reduces the
  boxed natural residual.
- `BM_LcpCompare` enables a 5-iteration PGS warm start for
  `BoxedSemiSmoothNewton` only on bounded/findex comparison packets; standard
  rows stay at the default zero warm-start iterations.
- Benchmark counters now expose
  `boxed_ssn_pgs_warm_start_iterations` and
  `boxed_ssn_pgs_warm_start_relaxation`.
- dartpy parameter bindings, C++ unit tests, Python tests, demo metadata,
  background docs, profile CSV artifacts, and `CHANGELOG.md` were updated to
  describe or cover the new parameters and refreshed profile summary.

Focused benchmark evidence:

- Focused pre-warm-start baseline for
  `BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BoxedSemiSmoothNewton/` was
  saved in `build/boxed_ssn_jacobian_copy_before.json`.
- Focused after-run kept standard rows neutral and substantially improved
  bounded/findex rows:
  - Standard ratios stayed around neutral: `0.976`, `1.009`, `0.954`, `0.999`.
  - Boxed ratios improved to `0.383`, `0.220`, `0.113` with 2 Newton
    iterations and `pgs=5`.
  - FrictionIndex ratios improved to `0.336`, `0.090`, `0.017` with 2 Newton
    iterations and `pgs=5`.
  - Focused mean ratio was `0.5096`; best `0.0174`; worst `1.0086`.
  - All focused after rows reported `contract_ok=1.0`.
- Full profile regeneration completed and refreshed the checked CSV artifacts.
  Snapshot after warm-start:
  - Standard: `BoxedSemiSmoothNewton` average ratio `3.24`.
  - Boxed: `BoxedSemiSmoothNewton` average ratio `3.67`, down from about
    `13.44`.
  - FrictionIndex: `BoxedSemiSmoothNewton` average ratio `3.47`, down from
    about `48.88`.

Verification completed for this slice:

```bash
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
pixi run build
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  python/tests/unit/math/test_lcp.py::test_advanced_boxed_solver_parameters_round_trip_from_dartpy_math \
  python/tests/unit/math/test_lcp.py::test_customized_advanced_boxed_solvers_solve_boxed_problem \
  -q
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_validation_and_solvers|all_solvers_smoke)$' \
  -j "$JOBS"
pixi run lint
git diff --check
```

Observed results:

- Cached profile replay completed and wrote
  `build/lcp_profile_full_check/performance_profile_*.csv`.
- CSV shape check reported 15 Boxed solver columns, 16 FrictionIndex solver
  columns, 23 Standard solver columns, and 200 rows per profile.
- `pixi run build` passed and rebuilt the dartpy module.
- Focused Python tests passed: `5 passed in 0.33s`.
- Focused CTests passed:
  `100% tests passed, 0 tests failed out of 2`.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step:

1. Continue benchmark-driven optimization or interface audit from the refreshed
   profile.
2. Push only after explicit maintainer/user approval.

Updated remaining profile targets after the warm-start evidence:

- Standard: `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `Admm`, `ShockPropagation`, `Nncg`, `BGS`, and `BlockedJacobi`.
- FrictionIndex: `BlockedJacobi`, `Staggering`, `BGS`,
  `ShockPropagation`, and `SubspaceMinimization`.

## 2026-06-12 Current Continuation - Post-SAP Checkpoint

Current branch state after the SAP slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Latest local checkpoint:
  `64777bcf7da Tune SAP boxed profile regularization`.
- Tracking state after the checkpoint: ahead of
  `origin/feature/lcp-solver-interface-demos` by 36 commits.
- Worktree was clean before this hand-off update.
- No PR is associated with this branch yet.
- No push has been performed in this continuation; pushes still require
  explicit maintainer/user approval.

Discarded follow-up investigation:

- Tried changing `BoxedSemiSmoothNewtonSolver` Jacobian assembly from
  `J.setZero()` plus interior-row copies to `J = A` plus active-row rewrites.
- Focused before/after
  `BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BoxedSemiSmoothNewton/` showed
  a clear regression: mean ratio `1.844`, best `1.080`, worst `2.271`, with
  every row slower.
- The draft was reverted completely and should not be reattempted without new
  evidence.
- The line-search sweep also did not justify a broad default change: more
  search steps only helped the 16-contact friction-index sweep row modestly
  while leaving other rows unchanged or slower.

Immediate next step:

- Continue from the refreshed profile after the SAP checkpoint. The clearest
  remaining targets are Standard `Lemke`, `InteriorPoint`, `Baraff`, and
  Newton-family rows; Boxed `Admm`, `BoxedSemiSmoothNewton`, and
  `ShockPropagation`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `BGS`, `ShockPropagation`, and `Staggering`. For
  `BoxedSemiSmoothNewton`, avoid the reverted Jacobian-copy approach above.

## 2026-06-12 Current Continuation - SAP Boxed Profile Regularization

After the ADMM checkpoint below, work continued on the next high-ratio bounded
profile target: `SapSolver` on boxed rows. The production solver default was
already `regularization = 1e-4`, but the main `BM_LcpCompare/*/Sap` profile was
overriding SAP to `1e-6` for every family. The focused regularization sweep
showed that boxed rows meet the comparison contract at `1e-4` and drop from
52 iterations to 3 on the 24-row boxed fixture, while standard and
friction-index rows were better kept at `1e-6`.

Current slice:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now configures SAP compare
  rows with `regularization = 1e-4` only for bounded non-findex packets, and
  keeps `regularization = 1e-6` for standard and friction-index packets.
- SAP benchmark counters now record `sap_regularization` and
  `sap_max_line_search_iterations` for main compare rows as well as contact
  rows, making cached benchmark JSON self-describing.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated.
- `docs/background/lcp/06_other-methods.md` documents the profile policy and
  keeps the regularization sweep as the source of evidence for stricter values.
- The Python demo profile summary and panel test were updated for the refreshed
  Standard and Boxed leader/laggard text.
- `CHANGELOG.md` records the SAP profile tuning.

Focused SAP before/after `BM_LcpCompare` evidence:

- Standard profile rows stayed effectively unchanged: runtime ratios `1.026`,
  `1.055`, `1.084`, and `1.057` for sizes 12, 24, 48, and 96 relative to the
  pre-change focused run; all still used `sap_regularization = 1e-6`.
- Boxed profile rows improved sharply: runtime ratios `0.060`, `0.038`, and
  `0.028` for sizes 12, 24, and 48; iterations changed from 34/52/70 to
  3/3/3; all used `sap_regularization = 1e-4`.
- FrictionIndex profile rows stayed effectively unchanged or improved within
  benchmark noise: runtime ratios `1.002`, `0.765`, and `0.956`; all still used
  `sap_regularization = 1e-6`.
- Mean focused ratio was `0.707`; best `0.028`; worst `1.084`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `Sap` average ratio is now `1.19`; current leaders are `Tgs`,
  `Pgs`, `Sap`, and tiny-row `Direct`; current high-ratio targets remain
  `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `Sap` average ratio improved from about `60.35` to `2.53`; current
  leaders are `Tgs`, `Pgs`, and `Jacobi`; current high-ratio targets are
  `Admm`, `BoxedSemiSmoothNewton`, and `ShockPropagation`.
- FrictionIndex: `Sap` average ratio is now `1.30`; current leaders are `Pgs`,
  `Tgs`, `Sap`, `SymmetricPsor`, and `Jacobi`; current high-ratio targets are
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpSapRegularizationSweep/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/sap_regularization_sweep_current.json
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Sap/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/sap_compare_before.json
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Sap/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/sap_compare_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- `BM_LCP_COMPARE` rebuilt successfully after the benchmark option change.
- The focused regularization sweep confirmed boxed `Reg1e_4` was the fast
  contract-passing row for the current 24-row boxed fixture.
- Focused SAP after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.40s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Checkpoint this SAP profile tuning slice. After that, continue
  benchmark-driven optimization or interface audit from the refreshed profile.
  The clearest remaining profile targets are Standard `Lemke`, `InteriorPoint`,
  `Baraff`, and Newton-family rows; Boxed `Admm`,
  `BoxedSemiSmoothNewton`, and `ShockPropagation`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

## 2026-06-12 Current Continuation - ADMM Default Rho Tuning

After the ShockPropagation checkpoint below, work continued from the refreshed
profile. A first attempt targeted `BoxedSemiSmoothNewtonSolver` by preallocating
line-search residual workspaces and skipping non-findex trial-bound copies, but
focused before/after benchmarking showed it was not worth keeping: mean ratio
`1.011`, best `0.878`, worst `1.082`, with large Standard and FrictionIndex
rows regressing. That draft was manually reverted before switching targets.

Current slice:

- `dart/math/lcp/other/admm_solver.hpp` now defaults
  `AdmmSolver::Parameters::rhoInit` to `4.0`.
- The change keeps the same projection, proximal linear solve, and adaptive
  residual-balancing algorithm; only the initial penalty for the existing
  adaptive path changes.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` now checks the new
  ADMM default parameter.
- `docs/background/lcp/06_other-methods.md` documents the default-rho decision
  and points to the DART-owned adaptive-rho sweep evidence.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated.
- The Python demo profile summary and panel test were updated for the refreshed
  Standard and Boxed leader/laggard text.
- `CHANGELOG.md` records the ADMM tuning.

Focused ADMM before/after `BM_LcpCompare` evidence:

- Standard profile rows: runtime ratios `0.806`, `0.653`, `0.566`, and `0.543`
  for sizes 12, 24, 48, and 96 relative to the previous default-rho profile
  cache.
- Boxed profile rows: runtime ratios `0.360`, `0.373`, and `0.364` for sizes
  12, 24, and 48.
- FrictionIndex profile rows: runtime ratios `0.903`, `0.548`, and `0.446` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.556`; best `0.360`; worst `0.903`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `Admm` average ratio is now `4.05`; current leaders are `Tgs`,
  tiny-row `Direct`, `Sap`, and `Pgs`; current high-ratio targets include
  `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `Admm` average ratio improved from about `31.06` to `14.51`; current
  leaders are `Pgs`, `Jacobi`, `Tgs`, and `SymmetricPsor`; current high-ratio
  targets are `Sap`, `Admm`, `BoxedSemiSmoothNewton`, and `ShockPropagation`.
- FrictionIndex: `Admm` average ratio is now `2.32`; current leaders are `Tgs`,
  `Pgs`, `Sap`, `SymmetricPsor`, and `Jacobi`; current high-ratio targets are
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/Admm/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/admm_profile_rho4_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- Comparison, generated coverage, and validation solver tests passed through
  CTest: `100% tests passed, 0 tests failed out of 3`.
- Focused ADMM after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.41s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. The clearest remaining profile targets are Standard `Lemke`,
  `InteriorPoint`, `Baraff`, and Newton-family rows; Boxed `Sap`,
  `BoxedSemiSmoothNewton`, and `ShockPropagation`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `ShockPropagation`, and
  `Staggering`.

## 2026-06-12 Current Continuation - ShockPropagation Feasible-Block Fast Path

The goal was resumed after the stop-only hand-off below. Work continued on the
next high-ratio profile target: `ShockPropagationSolver`.

Current slice:

- `dart/math/lcp/other/shock_propagation_solver.cpp` now tries a small
  `Eigen::FullPivLU` local solve for uncoupled fixed-bound blocks when the
  unconstrained candidate is finite and already feasible.
- Active-bound, singular, non-finite, larger-than-direct, and local `findex`
  blocks still use the existing Direct/Dantzig local subproblem fallback.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` covers a feasible
  boxed 3-row block that should solve directly in one outer iteration.
- `tests/unit/math/lcp/test_additional_solvers.cpp` keeps the non-standard
  fallback test active by forcing an upper-bound clamp.
- `docs/background/lcp/06_other-methods.md` documents the feasible-block fast
  path.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- The Python demo profile summary and panel test were updated for the refreshed
  Standard and FrictionIndex leader/laggard text.
- `CHANGELOG.md` records the `ShockPropagationSolver` optimization.

Focused before/after `BM_LcpCompare` evidence for this slice:

- Standard profile rows: runtime ratios `0.096`, `0.187`, `0.182`, and `0.196`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `1.031`, `0.651`, and `0.614` for sizes
  12, 24, and 48; the smallest boxed row regressed within benchmark noise while
  larger rows improved.
- FrictionIndex profile rows: runtime ratios `0.463`, `0.586`, and `0.777` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.478`; best `0.096`; worst `1.031`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `ShockPropagation` average ratio is now `7.75`; current leaders are
  `Pgs`, tiny-row `Direct`, `Tgs`, and `SymmetricPsor`; current high-ratio
  targets include `Lemke`, `InteriorPoint`, `Baraff`, and Newton-family rows.
- Boxed: `ShockPropagation` average ratio is now `19.70`; current leaders are
  `Jacobi`, `Tgs`, `Apgd`, and `SymmetricPsor`; current high-ratio targets are
  `Sap`, `Admm`, `BoxedSemiSmoothNewton`, and `ShockPropagation`.
- FrictionIndex: `ShockPropagation` average ratio is now `6.94`; current
  leaders are `Tgs`, `Sap`, `Pgs`, `SymmetricPsor`, and `Jacobi`; current
  high-ratio targets are `BoxedSemiSmoothNewton`, `BlockedJacobi`,
  `Staggering`, `BGS`, and `ShockPropagation`.

Verification completed so far:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_additional_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(additional_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/ShockPropagation/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/shock_profile_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- The first corrected build attempt failed because `Eigen::FullPivLU` needed
  `#include <Eigen/LU>`; that include was added and the corrected build/test
  target set was rerun successfully.
- Affected C++ targets rebuilt successfully.
- Additional solver, comparison, generated coverage, and validation solver
  tests passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused ShockPropagation after-run showed all Standard, Boxed, and
  FrictionIndex profile rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.48s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. The clearest remaining profile targets are Standard `Lemke`,
  `InteriorPoint`, `Baraff`, Newton-family rows, and `BlockedJacobi`; Boxed
  `Sap`, `Admm`, and `BoxedSemiSmoothNewton`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `Staggering`, and `BGS`.

## 2026-06-12 Stop-Only Hand-Off - Interrupted ShockPropagation Draft

The user explicitly stopped all further work and requested only hand-off docs,
with no further verification. Treat this as an interrupted state, not a
completed checkpoint. Do not continue implementation, verification, merging,
committing, pushing, or PR work unless the user explicitly asks to resume.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `8d42442e52c Optimize BGS singleton LCP blocks`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 33]`.
- Pre-existing worktree edits were present in three C++/test files:
  `dart/math/lcp/other/shock_propagation_solver.cpp`,
  `tests/unit/math/lcp/test_additional_solvers.cpp`, and
  `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`.
- This hand-off edit intentionally did not merge `main`, create a branch,
  commit, push, open/update a PR, or run any additional verification.

Uncommitted ShockPropagation draft currently in the worktree:

- `ShockPropagationSolver` has a new `solveUnconstrainedFeasibleBlock()` draft
  helper that tries a small `Eigen::FullPivLU` local solve for finite,
  uncoupled fixed-bound blocks when the unconstrained candidate is already
  feasible. Active-bound, singular, non-finite, larger-than-direct, and local
  `findex` blocks fall back to the existing Direct/Dantzig path.
- The solver loop calls that helper before constructing the local subproblem.
- `test_lcp_validation_and_solvers.cpp` adds
  `ShockPropagationSolverCoverage.SolvesFeasibleBoxedBlockDirectly`.
- `test_additional_solvers.cpp` adjusts
  `ShockPropagationSolver.NonStandardBlockUsesDantzig` so the boxed row clamps
  at the upper bound, preserving fallback coverage if the feasible-block fast
  path is kept.

Verification state for the interrupted draft:

- A focused ShockPropagation baseline benchmark was run before these draft
  edits and wrote `build/shock_profile_before.json`.
- A first focused build/test attempt after the draft edits used the wrong CMake
  target name, `UNIT_math_lcp_math_lcp_lcp_additional_solvers`. Ninja reported
  that target as unknown. Because that command did not fail fast, three older
  CTests still ran and passed; do not treat that as validation of the current
  draft.
- The correct target/test name discovered for the additional solver test is
  `UNIT_math_lcp_math_lcp_additional_solvers`.
- No correct compile, CTest, benchmark, lint, build, or `git diff --check` was
  run after the stop request or for this docs-only hand-off.

If a fresh session is explicitly asked to resume, first inspect the current
branch/worktree and reread `AGENTS.md`, `docs/ai/principles.md`,
`docs/dev_tasks/README.md`, this file, and `RESUME.md`. Then decide whether to
keep, revise, or remove the unverified ShockPropagation draft. If keeping it,
the first required validation target set is:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_additional_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(additional_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
```

Watch for a possible missing Eigen LU include around `Eigen::FullPivLU`. If the
draft compiles, run the focused ShockPropagation before/after benchmark before
updating profile artifacts or docs. Do not push without explicit approval.

## 2026-06-12 Current Continuation - BGS Singleton Fast Path

The goal was resumed after the stop-only hand-off below. Work continued on the
next high-ratio profile target: `BgsSolver`.

Current slice:

- `dart/math/lcp/projection/bgs_solver.cpp` now detects singleton blocks with
  no local `findex` coupling and solves the scalar fixed-bound subproblem with
  a direct projected Gauss-Seidel update.
- The fast path requires a finite positive diagonal and finite effective RHS.
  Non-singleton blocks, local `findex` blocks, non-positive diagonals, invalid
  scalar data, and coupled friction blocks still use the existing
  `DirectSolver`/`DantzigSolver` local block fallback.
- `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp` now covers BGS
  singleton boxed blocks that clamp against both lower and upper bounds.
- `docs/background/lcp/04_projection-methods.md` documents the singleton
  scalar BGS path.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- The Python demo profile summary and panel test were updated for the refreshed
  Boxed and FrictionIndex leader/laggard text.
- `CHANGELOG.md` records the `BgsSolver` optimization.

Focused before/after `BM_LcpCompare` evidence for this slice:

- Standard profile rows: runtime ratios `0.133`, `0.148`, `0.180`, and `0.292`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `0.115`, `0.123`, and `0.098` for sizes
  12, 24, and 48.
- FrictionIndex profile rows: runtime ratios `0.623`, `0.815`, and `0.982` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.351`; best `0.098`; worst `0.982`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `BGS` average ratio is now `2.42`. Current high-ratio targets remain
  `Lemke`, `InteriorPoint`, `Baraff`, and `ShockPropagation`.
- Boxed: `BGS` average ratio is now `4.22`, no longer in the top laggard set.
  Current high-ratio targets are `Sap`, `Admm`, `BoxedSemiSmoothNewton`, and
  `ShockPropagation`.
- FrictionIndex: `BGS` average ratio is now `7.16`; current high-ratio targets
  are `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, and
  `SubspaceMinimization`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_projection_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_projection_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BGS/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/bgs_profile_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- LCP projection, comparison, generated coverage, and validation solver tests
  passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused BGS after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- Focused Python LCP panel metadata test passed: `1 passed in 0.48s`.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. The clearest remaining profile targets are Standard `Lemke`,
  `InteriorPoint`, `Baraff`, `ShockPropagation`; Boxed `Sap`, `Admm`,
  `BoxedSemiSmoothNewton`, `ShockPropagation`; and FrictionIndex
  `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`, `SubspaceMinimization`.
- Do not push without explicit approval.

## 2026-06-12 Stop-Only Hand-Off - After BlockedJacobi Snapshot Product

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. Do not infer that lint, tests, builds, benchmarks,
implementation edits, commits, pushes, or PR work should continue from this
session. Resume implementation only after an explicit user request.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `4823706f0c9 Reuse BlockedJacobi fixed-bound snapshot products`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 32]`.
- Worktree: clean before this docs-only hand-off edit.
- Push was not performed in this stop-only response.
- No lint, test, build, benchmark, or other verification command was run after
  the stop request.

Latest completed local checkpoints:

- `4823706f0c9 Reuse BlockedJacobi fixed-bound snapshot products`
- `e7ea201e258 Optimize BlockedJacobi singleton LCP blocks`
- `3667401ad1f Optimize boxed semi-smooth Newton LCP step`
- `d27a0232c37 Show LCP performance profiles in py demo`
- `f857a380c20 Regenerate LCP performance profiles`
- `dcf0d835c1b Refresh LCP performance profile tooling`
- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`

Immediate next step:

- Stop. In a fresh session, first inspect the current branch/worktree state and
  reread `AGENTS.md`, `docs/ai/principles.md`, `docs/dev_tasks/README.md`,
  this file, and `RESUME.md`.
- The latest completed implementation slice is the BlockedJacobi snapshot
  product section immediately below. Its verification evidence is historical
  evidence from before this stop request, not work performed after it.
- The broad LCP solver/interface/demo goal remains incomplete. Existing profile
  data still suggests high-ratio future targets such as Standard `Lemke`,
  `InteriorPoint`, `Baraff`, `ShockPropagation`; Boxed `Sap`, `Admm`, `BGS`,
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `ShockPropagation`, `Staggering`, `BGS`. Do not start any
  of that work until explicitly asked.
- Do not push without explicit approval.

## 2026-06-12 Current Continuation - BlockedJacobi Snapshot Product

After the singleton fast path checkpoint, work continued on the same
`BlockedJacobiSolver` profile target. The remaining hot loop repeated
`A.row(i).dot(xPrev)` inside every block update. Fixed-bound Standard and Boxed
problems now precompute the Jacobi snapshot product `A * xPrev` once per
iteration and reuse it for block right-hand sides. Coupled friction-index
problems keep the prior row-product path because an unconditional snapshot
slightly slowed those rows during focused testing.

Current slice:

- `dart/math/lcp/projection/blocked_jacobi_solver.cpp` passes an optional
  snapshot product into `solveBlock()`.
- Standard and Boxed fixed-bound problems use the snapshot product for singleton
  and non-singleton block RHS construction.
- Friction-index problems continue using per-row products inside their coupled
  block path.
- `docs/background/lcp/06_other-methods.md` documents the fixed-bound snapshot
  product.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated again.
- The Python demo profile summary and panel test were adjusted for the refreshed
  Boxed and FrictionIndex leader/laggard text.
- `CHANGELOG.md` records the follow-up `BlockedJacobiSolver` optimization.

Focused before/after `BM_LcpCompare` evidence for this slice:

- Standard profile rows: runtime ratios `0.989`, `0.838`, `0.811`, and `0.654`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `1.023`, `0.923`, and `0.832` for sizes
  12, 24, and 48; the smallest row is within benchmark noise while larger rows
  improve.
- FrictionIndex profile rows: runtime ratios `0.912`, `0.858`, and `0.904` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.874`; best `0.654`; worst `1.023`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `BlockedJacobi` average ratio improved from `5.66` to `4.60`;
  current leaders are `Tgs`/`Sap`/`Pgs` on scalable rows and `Direct` on tiny
  rows.
- Boxed: `BlockedJacobi` average ratio improved from `3.59` to `3.30`; current
  leader is `Tgs`, with `Pgs`/`Jacobi` close.
- FrictionIndex: `BlockedJacobi` average ratio improved from `20.45` to
  `15.08`; it remains a future optimization target.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_projection_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_projection_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BlockedJacobi/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/blocked_jacobi_profile_ax_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- LCP projection, comparison, generated coverage, and validation solver tests
  passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused BlockedJacobi after-run showed all Standard, Boxed, and
  FrictionIndex profile rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Focused Python LCP panel metadata test passed: `1 passed in 0.60s`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. Fresh high-ratio targets include Standard `Lemke`, `InteriorPoint`,
  `Baraff`, and `ShockPropagation`; Boxed `Sap`, `Admm`, `BGS`, and
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `ShockPropagation`, `Staggering`, and `BGS`.
- Do not push without explicit approval.

## 2026-06-12 Current Continuation - BlockedJacobi Singleton Fast Path

After the stop-only hand-off below, work resumed on the selected
benchmark-driven target: `BlockedJacobiSolver`.

Current slice:

- `dart/math/lcp/projection/blocked_jacobi_solver.cpp` now detects singleton
  blocks with no local `findex` coupling and solves the scalar boxed subproblem
  with a direct projected update.
- The fast path requires a finite positive diagonal and finite effective RHS.
  Non-singleton blocks, local `findex` blocks, non-positive diagonals, and
  invalid scalar data still use the existing `DirectSolver`/`DantzigSolver`
  fallback path.
- `tests/unit/math/lcp/test_lcp_projection_solvers.cpp` now covers singleton
  boxed blocks that clamp against both lower and upper bounds.
- `docs/background/lcp/06_other-methods.md` documents the scalar singleton
  route and corrects the Jacobi block RHS sign in the pseudocode.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- `python/examples/demos/scenes/lcp_physics.py` and the panel metadata test now
  reflect the refreshed current leader/laggard families.
- `CHANGELOG.md` records the solver optimization.

Focused before/after `BM_LcpCompare` evidence for `BlockedJacobiSolver`:

- Standard profile rows: runtime ratios `0.088`, `0.085`, `0.134`, and `0.169`
  for sizes 12, 24, 48, and 96 relative to the pre-change focused run.
- Boxed profile rows: runtime ratios `0.069`, `0.082`, and `0.093` for sizes
  12, 24, and 48.
- FrictionIndex profile rows: runtime ratios `0.734`, `0.623`, and `0.661` for
  contact counts 4, 16, and 64.
- Mean focused ratio was `0.274`; best `0.069`; worst `0.734`.
- All focused after rows reported `contract_ok=1.0`.

Final regenerated profile snapshot:

- Standard: `BlockedJacobi` average ratio `5.66`; current leaders are
  `Pgs`/`Sap`/`Tgs` on scalable rows and `Direct` on tiny rows.
- Boxed: `BlockedJacobi` average ratio `3.59`; current leaders are `Pgs`/`Tgs`
  with `Jacobi` close.
- FrictionIndex: `BlockedJacobi` average ratio `20.45`; this remains a future
  optimization target because coupled contact blocks still use the local block
  solve path.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target BM_LCP_COMPARE \
           UNIT_math_lcp_math_lcp_lcp_projection_solvers \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_projection_solvers|lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BlockedJacobi/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/blocked_jacobi_profile_after.json
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
pixi run python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
pixi run build
pixi run lint
git diff --check
```

Observed results:

- Affected C++ targets rebuilt successfully.
- LCP projection, comparison, generated coverage, and validation solver tests
  passed through CTest: `100% tests passed, 0 tests failed out of 4`.
- Focused BlockedJacobi after-run showed all Standard, Boxed, and
  FrictionIndex profile rows passing contract checks.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Focused Python LCP panel metadata test passed: `1 passed in 0.78s`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.
- `pixi run build` passed.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization or interface audit from the refreshed
  profile. Fresh high-ratio targets include Standard `Lemke`, `InteriorPoint`,
  `Baraff`, and `ShockPropagation`; Boxed `Sap`, `BGS`, `Admm`, and
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, `ShockPropagation`, and `BGS`.
- Do not push without explicit approval.

## 2026-06-12 Stop-Only Hand-Off - After BlockedJacobi Recon

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. Do not infer that lint, tests, builds, benchmarks,
implementation edits, commits, or pushes were performed after that stop request.
This stop-only update is limited to this file and
`docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `3667401ad1f Optimize boxed semi-smooth Newton LCP step`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 30]`.
- Worktree: clean before this docs-only hand-off edit.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `3667401ad1f Optimize boxed semi-smooth Newton LCP step`
- `d27a0232c37 Show LCP performance profiles in py demo`
- `f857a380c20 Regenerate LCP performance profiles`
- `dcf0d835c1b Refresh LCP performance profile tooling`
- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`

Interrupted continuation slice:

- The session selected `BlockedJacobiSolver` as the next benchmark-driven target
  because the refreshed profiles still show it as a high-ratio family across
  Standard, Boxed, and FrictionIndex rows.
- Context was gathered from the BlockedJacobi implementation/header, LCP
  overview/method docs, and related unit-test snippets.
- No BlockedJacobi implementation files were edited.
- No focused BlockedJacobi baseline benchmark was run.
- No lint, test, build, benchmark regeneration, commit, or push was run after
  the stop request.

Immediate next step:

- Do nothing until the user explicitly resumes implementation. On resume, first
  inspect the current branch state and reread this task README and `RESUME.md`.
  If continuing the selected path, inspect the remaining BlockedJacobi update
  loop and measure a focused `BlockedJacobi` baseline before considering a
  singleton-block fast path. Do not push without explicit approval.

## 2026-06-12 Current Continuation - Boxed Semi-Smooth Newton Direct Step

After the Python demo performance-profile surface, work moved into the next
benchmark-driven optimization target: `BoxedSemiSmoothNewtonSolver`.

Current slice:

- `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp` now tries the
  direct semi-smooth Newton system `J dx = -H` before falling back to the
  previous regularized least-squares normal-equation solve.
- Line-search trials now keep friction-index moving-bound updates local to the
  trial state, and only publish the updated effective bounds when a step is
  accepted.
- The checked performance-profile CSV artifacts under
  `docs/background/lcp/figures` were regenerated from the optimized solver.
- `docs/background/lcp/05_newton-methods.md` now documents the direct
  Jacobian-solve common path and least-squares fallback.
- `python/examples/demos/scenes/lcp_physics.py` and the panel metadata test now
  reflect the refreshed current leader/laggard families after the optimization.
- `CHANGELOG.md` records the solver optimization.

Focused before/after `BM_LcpCompare` evidence for
`BoxedSemiSmoothNewtonSolver`:

- Standard profile rows: mean runtime ratio `0.577`, best `0.395`, worst
  `0.731` relative to the pre-change focused run.
- Boxed profile rows: mean runtime ratio `0.608`, best `0.502`, worst `0.696`.
- FrictionIndex profile rows: mean runtime ratio `0.557`, best `0.451`, worst
  `0.704`.
- All focused after rows reported `contract_ok=1.0`.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
build/default/cpp/Release/bin/BM_LCP_COMPARE \
  --benchmark_filter='BM_LcpCompare/(Standard|Boxed|FrictionIndex)/BoxedSemiSmoothNewton/' \
  --benchmark_min_time=0.01s \
  --benchmark_format=json > build/bssn_profile_after.json
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           UNIT_math_lcp_math_lcp_lcp_generated_coverage \
           UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R 'UNIT_math_lcp_math_lcp_(lcp_comparison_harness|lcp_generated_coverage|lcp_validation_and_solvers)$' \
  -j "$JOBS"
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
pixi run python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows)
PY
```

Observed results:

- `BM_LCP_COMPARE` rebuilt successfully.
- Focused BSSN after-run showed all Standard, Boxed, and FrictionIndex profile
  rows passing contract checks and running at roughly 40-70% of the pre-change
  runtime, depending on row.
- LCP comparison, generated coverage, and validation solver test executables
  all passed through CTest: `100% tests passed, 0 tests failed out of 3`.
- Full `BM_LcpCompare/` profile regeneration completed and updated all checked
  CSV artifacts.
- Cached profile replay completed under `build/lcp_profile_full_check`.
- Focused Python LCP panel metadata test passed: `1 passed in 0.47s`.
- Generated CSV shape check reported 23 Standard solver columns, 15 Boxed
  solver columns, 16 FrictionIndex solver columns, and 200 rows per profile.

Immediate next step after this checkpoint:

- Continue benchmark-driven optimization. The freshest full profile still shows
  high-ratio targets:
  - Standard: `Lemke`, `InteriorPoint`, `Baraff`, `BlockedJacobi`,
    `ShockPropagation`.
  - Boxed: `Sap`, `BlockedJacobi`, `BGS`, `Admm`, `BoxedSemiSmoothNewton`.
  - FrictionIndex: `BoxedSemiSmoothNewton`, `BlockedJacobi`, `BGS`,
    `Staggering`, `ShockPropagation`.
- Pick one solver/problem family, measure it with the same profile tooling,
  optimize or retune it, then update benchmark/demo evidence. Do not push
  without explicit approval.

## 2026-06-12 Current Continuation - Python Demo Performance Profiles

After the stop-only hand-off and full profile refresh, work resumed on the next
demo-focused gap: making the refreshed performance-profile artifacts visible
from the Python LCP demo so users do not have to manually parse benchmark CSVs
to understand current solver tradeoffs.

Current slice:

- `python/examples/demos/scenes/lcp_physics.py` now exposes
  `performance_profile_rows` and `performance_profile_refresh_command` in the
  scene `info` metadata.
- The `lcp_physics` panel now has a "Performance profiles" table pointing to
  the checked Standard, Boxed, and FrictionIndex CSV artifacts under
  `docs/background/lcp/figures`.
- The new table summarizes the current profile row families, problem sizes,
  leader families, laggard families, and practical takeaways:
  - Standard: scalable rows currently favor TGS/PGS, while Direct is only a
    tiny-problem leader and Lemke/InteriorPoint/Baraff/BlockedJacobi remain
    high-ratio targets.
  - Boxed: active-bound rows currently favor Jacobi/PGS, while SAP,
    BlockedJacobi, and boxed semi-smooth Newton remain tuning/optimization
    targets.
  - FrictionIndex: contact-scale rows currently favor TGS/SAP/PGS, while boxed
    semi-smooth Newton, BlockedJacobi, and ShockPropagation remain
    tuning/optimization targets.
- `python/tests/unit/test_py_demo_panels.py` now asserts the profile metadata,
  artifacts, refresh command, leaders/laggards, and panel table registration.
- `python/examples/demos/README.md` documents that the panel surfaces checked
  performance-profile CSV summaries and includes the full refresh command.
- `CHANGELOG.md` records the new Python demo performance-profile surface.

Verification completed for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest \
  python/tests/unit/test_py_demo_panels.py::test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata \
  -q
pixi run lint
git diff --check
```

Observed results:

- Focused LCP panel metadata test passed: `1 passed in 0.43s`.
- `pixi run lint` passed, including the LCP solver roster gate.
- `git diff --check` passed.

Immediate next step after this checkpoint:

- Continue the broad objective with benchmark-driven optimization or a deeper
  solver-interface audit. The freshest actionable performance targets are the
  high-ratio rows surfaced in the demo summary: Standard
  Lemke/InteriorPoint/Baraff/BlockedJacobi, Boxed
  SAP/BlockedJacobi/BoxedSemiSmoothNewton, and FrictionIndex
  BoxedSemiSmoothNewton/BlockedJacobi/ShockPropagation. Pick one concrete
  solver/problem family, measure it with the checked profile tooling, optimize
  or retune it, then update the matching benchmark/demo evidence. Do not push
  without explicit approval.

## 2026-06-12 Stop-Only Hand-Off - After Full Profile Refresh

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. Do not infer that lint, tests, builds, benchmark
listing, benchmark regeneration, implementation edits, commits, or pushes were
performed after that stop request. This stop-only update is limited to this
file and `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `f857a380c20 Regenerate LCP performance profiles`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 28]`.
- Worktree: clean before this docs-only hand-off edit.
- Remote tracking ref still pointed at
  `737b9c95c11 Document final LCP handoff checkpoint` when inspected by
  `git log --decorate`.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `f857a380c20 Regenerate LCP performance profiles`
- `dcf0d835c1b Refresh LCP performance profile tooling`
- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`
- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`

Interrupted/unstarted continuation slice:

- No implementation files were edited for the next slice after the stop
  request.
- No lint, tests, builds, benchmark listing, benchmark regeneration, commit, or
  push was run after the stop request.
- The session had selected, but not implemented, a likely next slice: expose
  the refreshed performance-profile leaders and laggards in the Python LCP demo
  metadata/panel so users can see solver pros/cons without manually parsing
  benchmark output.
- Context gathered before the stop request, from the current full profile cache,
  suggested the next explanatory/demo metadata should call out slow/high-ratio
  families such as Standard `Lemke`, `InteriorPoint`, `Baraff`, and
  `BlockedJacobi`; Boxed `Sap`, `BlockedJacobi`, and
  `BoxedSemiSmoothNewton`; and FrictionIndex `BoxedSemiSmoothNewton`,
  `BlockedJacobi`, and `ShockPropagation`. Treat this as hand-off context, not
  a completed product change.

Immediate next step:

- Do nothing until the user explicitly resumes implementation. On resume,
  inspect the current branch state, reread this task README and `RESUME.md`,
  then choose one concrete solver/interface/demo/performance gap. The most
  recent unstarted slice is the Python LCP demo performance-profile summary
  described above. Do not push without explicit approval.

## 2026-06-12 Current Continuation - Full Performance Profile Refresh

After checkpoint `dcf0d835c1b Refresh LCP performance profile tooling`, work
continued into the next benchmark-focused gap: regenerating the checked
performance-profile CSV artifacts from current benchmark rows.

Current slice:

- A first full script run exposed that the profile script was launching every
  benchmark in `BM_LCP_COMPARE`, including many unrelated stress/sweep rows
  ignored by the profile parser; that unfiltered run timed out after 600s.
- `scripts/lcp_performance_profile.py` now defaults benchmark execution to the
  profile row family `BM_LcpCompare/`.
- The script now exposes `--benchmark-timeout` and reports timeout failures with
  an actionable message instead of a raw Python traceback.
- `docs/background/lcp/figures/performance_profile_standard.csv`,
  `docs/background/lcp/figures/performance_profile_boxed.csv`, and
  `docs/background/lcp/figures/performance_profile_frictionindex.csv` were
  regenerated from a current full `BM_LcpCompare/` JSON packet.
- The generated CSVs now have 23 Standard solver columns, 15 Boxed solver
  columns, and 16 FrictionIndex solver columns, matching native manifest
  coverage.

Verification completed for this slice:

```bash
pixi run python scripts/lcp_performance_profile.py --run \
  --cache build/lcp_profile_full.json \
  --output docs/background/lcp/figures
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_full.json \
  --output build/lcp_profile_full_check
pixi run python scripts/check_lcp_solver_roster.py
pixi run python - <<'PY'
import csv
from pathlib import Path
for path in sorted(Path('docs/background/lcp/figures').glob('performance_profile_*.csv')):
    with path.open(newline='') as f:
        header = next(csv.reader(f))
        rows = sum(1 for _ in f)
    print(path.name, len(header) - 1, rows, header[:4], header[-3:])
PY
```

Observed results:

- The current full profile row family completed and cached results to
  `build/lcp_profile_full.json`.
- Cached replay completed and wrote check copies under
  `build/lcp_profile_full_check`.
- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- Generated CSV shape check reported:
  - `performance_profile_standard.csv`: 23 solver columns, 200 rows.
  - `performance_profile_boxed.csv`: 15 solver columns, 200 rows.
  - `performance_profile_frictionindex.csv`: 16 solver columns, 200 rows.

Immediate next step after this checkpoint:

- Continue the broad objective with the next concrete solver/interface/demo or
  performance gap. The refreshed profiles now make the next benchmark-driven
  optimization target visible: inspect the slow/high-ratio solver families and
  decide whether to improve implementation performance, tune benchmark
  parameters, or improve the py-demo explanations for solver pros/cons.

## 2026-06-12 Current Continuation - Performance Profile Regeneration Schema

After the stop-only hand-off, work resumed on the broad DART 7 LCP objective.
The next bounded gap was the profile-refresh path: the checked CSV headers were
already filtered through native support metadata, but the regeneration script
still parsed the old benchmark naming scheme and the `BM_LCP_COMPARE` benchmark
target still carried stale native support count assertions.

Current slice:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now asserts 24 total solvers,
  23 standard-native solvers, 15 boxed-native solvers, and 16
  friction-index-native solvers, matching the manifest and roster checker after
  Staggering stopped advertising native standard/boxed support.
- `scripts/lcp_performance_profile.py` now parses the current benchmark schema:
  `BM_LcpCompare/<problem-family>/<solver>/<problem-size>`.
- The profile script keeps compatibility with historical cached JSON using the
  old `BM_LcpCompare_<solver>_<problem-family>/<problem-size>` schema.
- The profile script now rejects full profile generation when benchmark JSON is
  missing native solver coverage for any Standard, Boxed, or FrictionIndex
  category. `--allow-partial` is available for explicit smoke runs.
- `--benchmark-filter` and `--benchmark-min-time` are forwarded to Google
  Benchmark so focused profile smoke packets can exercise the current parser
  without running the full comparison suite.
- Full current performance-profile artifacts were not regenerated in this
  slice; the next benchmark-refresh slice should run the full profile
  generation without a filter and decide whether to update the checked CSV/PNG
  artifacts from those measurements.

Verification completed for this slice:

```bash
pixi run python - <<'PY'
from scripts.lcp_performance_profile import (
    check_native_profile_coverage,
    load_native_support_by_category,
    parse_benchmark_results,
    parse_benchmark_name,
)

assert parse_benchmark_name('BM_LcpCompare/Standard/BGS/12') == ('Standard', 'BGS', 12)
assert parse_benchmark_name('BM_LcpCompare/Boxed/NNCG/24') == ('Boxed', 'NNCG', 24)
assert parse_benchmark_name('BM_LcpCompare_FischerBurmeisterNewton_Standard/12') == ('Standard', 'FischerBurmeisterNewton', 12)
assert parse_benchmark_name('BM_LcpCompare_Dantzig_Scaled/12/0') is None
parsed = parse_benchmark_results({
    'benchmarks': [
        {'name': 'BM_LcpCompare/Standard/Bgs/12', 'cpu_time': 2.0, 'contract_ok': 1.0},
        {'name': 'BM_LcpCompare/Boxed/Nncg/24', 'cpu_time': 3.0, 'contract_ok': 1.0},
        {'name': 'BM_LcpCompare/FrictionIndex/Mprgp/4', 'cpu_time': 4.0, 'contract_ok': 1.0},
        {'name': 'BM_LcpCompare/Standard/BGS/12_mean', 'run_type': 'aggregate'},
    ]
})
assert ('BGS', 12) in parsed['Standard']
assert ('NNCG', 24) in parsed['Boxed']
assert ('MPRGP', 4) in parsed['FrictionIndex']
try:
    check_native_profile_coverage(parsed, load_native_support_by_category())
except RuntimeError as exc:
    assert 'missing native solvers' in str(exc)
else:
    raise AssertionError('expected missing coverage failure')
check_native_profile_coverage(parsed, load_native_support_by_category(), allow_partial=True)
print('profile parser smoke passed')
PY
cmake --build build/default/cpp/Release --target BM_LCP_COMPARE --parallel "$JOBS"
pixi run python scripts/lcp_performance_profile.py --run \
  --benchmark-filter='BM_LcpCompare/(Standard|Boxed)/(Dantzig|Pgs)/12$' \
  --benchmark-min-time=0.001 \
  --cache build/lcp_profile_smoke.json \
  --output build/lcp_profile_smoke \
  --allow-partial
pixi run python scripts/lcp_performance_profile.py \
  --cache build/lcp_profile_smoke.json \
  --output build/lcp_profile_smoke_failcheck
pixi run python - <<'PY'
import subprocess
from scripts.lcp_performance_profile import (
    check_native_profile_coverage,
    load_native_support_by_category,
    parse_benchmark_name,
)

completed = subprocess.run(
    [
        'build/default/cpp/Release/bin/BM_LCP_COMPARE',
        '--benchmark_list_tests=true',
        '--benchmark_filter=BM_LcpCompare/',
    ],
    check=True,
    capture_output=True,
    text=True,
)
results = {}
for name in completed.stdout.splitlines():
    parsed = parse_benchmark_name(name.strip())
    if parsed is None:
        continue
    category, solver, problem_size = parsed
    results.setdefault(category, {})[(solver, problem_size)] = {
        'time_ns': 1.0,
        'contract_ok': 1.0,
    }
check_native_profile_coverage(results, load_native_support_by_category())
counts = {category: len({solver for solver, _ in rows}) for category, rows in results.items()}
print(counts)
PY
pixi run python scripts/check_lcp_solver_roster.py
git diff --check
pixi run lint
```

Observed results:

- Synthetic parser smoke passed and proved strict coverage fails on incomplete
  benchmark JSON unless `--allow-partial` is set.
- `BM_LCP_COMPARE` rebuilt successfully from current source.
- Focused real benchmark smoke produced partial Standard and Boxed profile CSVs
  under `build/lcp_profile_smoke`.
- Strict mode rejected that partial smoke cache, as expected, with missing
  native solver coverage diagnostics.
- The rebuilt benchmark registration list covered exactly
  `{'Standard': 23, 'Boxed': 15, 'FrictionIndex': 16}` native solvers.
- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- `git diff --check` passed.
- `pixi run lint` passed, including the LCP roster gate.

Immediate next step after this checkpoint:

- Continue the broad objective with another concrete solver/interface/demo or
  performance gap. A likely next benchmark-focused slice is to run the full
  unfiltered performance-profile generation from the rebuilt benchmark target,
  review the current measurements, and update the checked profile artifacts if
  the run is representative enough.

## 2026-06-12 Stop-Only Hand-Off - After Performance Profile Metadata

The user explicitly stopped further work again and requested only hand-off docs,
with no further verification. Do not infer that lint, tests, builds, benchmark
listing, commits, pushes, or implementation edits were performed after that
request. This stop-only update is limited to this file and
`docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `7df9a018c31 Align LCP performance profile metadata`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 26]`.
- Worktree: clean before this docs-only hand-off edit.
- No PR was associated with this branch when the already-running GitHub status
  command completed.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `7df9a018c31 Align LCP performance profile metadata`
- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`
- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`

Interrupted continuation slice:

- No implementation files were edited after the stop request.
- No lint, tests, builds, benchmark listing, benchmark regeneration, commit, or
  push was run after the stop request.
- The session had only reconstructed current context and closed out an
  already-running `gh pr status` command. That command reported no PR
  associated with `feature/lcp-solver-interface-demos`.
- The broad DART 7 LCP solver/interface/demo objective is not complete. The
  latest completed slice only aligned performance-profile tooling and checked
  CSV headers with native solver support metadata.

Immediate next step:

- Do nothing until the user explicitly resumes implementation. On resume,
  inspect the current branch state, reread this task README and `RESUME.md`,
  then choose one concrete solver/interface/demo/performance gap. A likely
  next gap remains regenerating performance-profile artifacts from current
  benchmark results rather than synthesizing missing solver columns from older
  cached data. Do not push without explicit approval.

## 2026-06-12 Current Continuation - LCP Performance Profile Native Support

After the stop-only hand-off and the Staggering capability-metadata checkpoint,
work resumed on the broad DART 7 LCP objective. The benchmark registration
audit found that the main benchmark registration paths already instantiate
solvers and call `supportsProblem(problem)` before registering concrete rows,
but the background performance-profile tooling and checked CSV artifacts still
allowed stale non-native solver columns.

Current slice:

- `scripts/lcp_performance_profile.py` now loads the checked C++ solver
  manifest and filters each Standard, Boxed, and FrictionIndex profile to
  solvers that are native for that problem form.
- The performance-profile parser canonicalizes acronym solver names such as
  `BGS`, `NNCG`, and `MPRGP` to the manifest spelling before computing profile
  ratios.
- `scripts/check_lcp_solver_roster.py` now checks the performance-profile CSV
  headers against native manifest support, catching stale columns such as
  delegated-only Staggering in standard or boxed profiles.
- `docs/background/lcp/figures/performance_profile_standard.csv` and
  `docs/background/lcp/figures/performance_profile_boxed.csv` no longer list
  Staggering as a native profile column. Existing cached measurements were not
  rerun in this slice, so missing newly supported profile columns remain a
  future benchmark-refresh task rather than synthesized data.
- `CHANGELOG.md` records the profile tooling/header alignment.

Verification completed for this slice so far:

```bash
pixi run python scripts/check_lcp_solver_roster.py
pixi run python - <<'PY'
from scripts.lcp_performance_profile import (
    compute_performance_ratios,
    parse_benchmark_results,
)

results = {
    "Standard": {
        ("NativeSolver", 2): {"time_ns": 1.0, "contract_ok": 1.0},
        ("DelegatedOnlySolver", 2): {"time_ns": 0.1, "contract_ok": 1.0},
    }
}
ratios, solvers, problems = compute_performance_ratios(
    results, "Standard", {"NativeSolver"}
)
assert solvers == ["NativeSolver"]
assert problems == [2]
assert ratios["NativeSolver"] == [1.0]
parsed = parse_benchmark_results({
    "benchmarks": [
        {"name": "BM_LcpCompare_Bgs_Standard/2", "cpu_time": 1.0, "contract_ok": 1.0},
        {"name": "BM_LcpCompare_Nncg_Boxed/2", "cpu_time": 1.0, "contract_ok": 1.0},
        {"name": "BM_LcpCompare_Mprgp_Standard/2", "cpu_time": 1.0, "contract_ok": 1.0},
    ]
})
assert ("BGS", 2) in parsed["Standard"]
assert ("NNCG", 2) in parsed["Boxed"]
assert ("MPRGP", 2) in parsed["Standard"]
print("performance profile native-support filter passed")
PY
pixi run lint
```

Observed results:

- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- Synthetic profile parsing/filter check passed.
- `pixi run lint` passed, including the updated LCP solver roster gate.

Immediate next step after this checkpoint:

- Continue the broad objective with another concrete solver/interface/demo or
  performance gap. A later benchmark-refresh slice should regenerate the
  performance-profile artifacts from current benchmark results rather than
  synthesizing missing solver columns from older cached data.

## 2026-06-12 Stop-Only Hand-Off - After Staggering Capability Metadata

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. Do not infer that lint, tests, builds, benchmark
listing, commits, pushes, or implementation edits were performed after that
request. This stop-only update is limited to this file and
`docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `00a58f7153e Align Staggering native capability metadata`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 25]`.
- Worktree: clean before this docs-only hand-off edit.
- No PR was associated with this branch when checked earlier in the session.
- Push was not performed in this stop-only response.

Latest completed local checkpoints:

- `00a58f7153e Align Staggering native capability metadata`
- `1b4e3827618 Report Staggering native support precisely`
- `0e75c5c3985 Validate remaining LCP solver parameters`
- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`

Interrupted continuation slice:

- No implementation files were edited in the interrupted continuation slice.
- The session was only reconstructing context and choosing the next bounded
  gap after the Staggering capability-method checkpoint.
- The likely next audit target was benchmark/demo registration and metadata
  after Staggering stopped advertising native standard/boxed support. Candidate
  files to inspect on explicit resume include
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`,
  `python/examples/demos/scenes/lcp_physics.py`, and
  `python/tests/unit/test_py_demo_panels.py`.
- The broad DART 7 LCP solver/interface/demo objective is not complete. The
  latest completed slice only aligned Staggering native capability metadata
  and related tests.

Immediate next step:

- Do nothing until the user explicitly resumes implementation. On resume,
  inspect the current branch state, reread this task README and `RESUME.md`,
  then choose one concrete solver/interface/demo/performance gap. Do not push
  without explicit approval.

## 2026-06-12 Current Continuation — Staggering Form-Level Native Capabilities

After checkpoint `1b4e3827618 Report Staggering native support precisely`, the
remaining interface mismatch was Staggering's form-level capability methods:
`supportsProblem(problem)` correctly reported standard and boxed no-friction
packets as delegated, but `supportsStandardLcp()` and `supportsBoxedLcp()`
still advertised native support.

Current slice:

- `StaggeringSolver::supportsStandardLcp()` and
  `StaggeringSolver::supportsBoxedLcp()` now return false.
- `StaggeringSolver::supportsFrictionIndex()` remains true, and
  `supportsProblem(problem)` still requires a friction-index packet with both
  normal and friction rows for native Staggering support.
- The C++ solver manifest, comparison-harness coverage table, dartpy LCP tests,
  Python LCP demo metadata, py-demo panel tests, and solver-roster checker now
  expect 24 total solvers, 23 standard-native solvers, 15 boxed-native solvers,
  and 16 friction-index-native solvers.
- Standard and boxed no-friction packets remain solvable through `solve()`
  fallback delegation.
- `CHANGELOG.md` records the capability-method alignment.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke \
           UNIT_math_lcp_math_lcp_lcp_comparison_harness \
           dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke \
  --gtest_filter='AllSolversSmokeTest.ManifestMatchesConstructedSolverMetadata:AllSolversSmokeTest.SolverCapabilityPredicatesClassifyProblemForms:AllSolversSmokeTest.SolverCapabilityPredicatesUseDefaultToleranceForNearStandardForm:AllSolversSmokeTest.StaggeringReportsOnlyFrictionBlockProblemsAsNative'
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_comparison_harness \
  --gtest_filter='LcpComparisonHarness.ManifestMatchesFixtureCoverage:LcpComparisonHarness.StaggeringOnStandardAndBoxedFixtures:LcpComparisonHarness.StaggeringOnFrictionIndexFixtures'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- Focused C++ all-solvers smoke coverage passed 4/4 tests.
- Focused C++ comparison-harness coverage passed 3/3 tests.
- `python/tests/unit/math/test_lcp.py`: 78 passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 passed.
- LCP solver roster check passed:
  `24 solvers, 23 standard, 15 boxed, 16 findex`.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the updated LCP solver roster gate.

Immediate next step after this checkpoint:

- Continue the broad DART 7 LCP objective with the next concrete solver,
  interface, demo, or performance gap. Do not mark the overall goal complete
  from this focused capability-method slice.

## 2026-06-12 Current Continuation — Staggering Native Support Reporting

After the stop-only hand-off, work resumed on the broad LCP solver/interface
goal. This slice tightens Staggering capability reporting so benchmark and demo
metadata distinguish native Staggering solves from Dantzig fallback solves.

Current slice:

- `StaggeringSolver::supportsProblem()` now reports native support only for
  friction-index packets containing both normal and friction rows.
- Standard, boxed no-friction, near-standard, and large standard packets now
  report false for Staggering native support because those paths are delegated,
  even though `solve()` can still succeed through the existing fallback path.
- C++ smoke coverage checks the standard, boxed, near-standard, large standard,
  and friction-index support predicates, including explicit fallback solve
  behavior on a standard packet.
- dartpy LCP tests mirror the C++ support-predicate expectations and keep a
  fallback solve success check for a standard packet.
- `CHANGELOG.md` records the native-support-reporting behavior.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke \
  --gtest_filter='AllSolversSmokeTest.SolverCapabilityPredicatesClassifyProblemForms:AllSolversSmokeTest.SolverCapabilityPredicatesUseDefaultToleranceForNearStandardForm:AllSolversSmokeTest.StaggeringReportsOnlyFrictionBlockProblemsAsNative:AllSolversSmokeTest.NearSingularStandardProblemProducesExpectedIterates'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The C++ smoke target and dartpy rebuilt successfully.
- Focused C++ smoke coverage passed 4/4 tests.
- `python/tests/unit/math/test_lcp.py`: 78 passed.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step after this checkpoint:

- Continue the broad DART 7 LCP objective with the next concrete solver,
  interface, demo, or performance gap. Do not mark the overall goal complete
  from this focused support-reporting slice.

## 2026-06-12 Current Continuation — Remaining Parameter Validation

The user resumed the broad LCP solver/interface/demo goal after the stop-only
hand-off. This slice turns the remaining parameter-validation audit into
implementation and focused coverage.

Current slice:

- `PgsSolver`, `ApgdSolver`, and `TgsSolver` now reject invalid
  `epsilonForDivision` before iteration. `ApgdSolver` also rejects negative
  `restartCheckInterval` while preserving `0` as the documented
  every-iteration restart-check mode.
- `MinimumMapNewtonSolver`, `FischerBurmeisterNewtonSolver`, and
  `PenalizedFischerBurmeisterNewtonSolver` now validate their exposed
  line-search, gradient-descent warm-start, PGS warm-start, smoothing, and
  penalty parameters before numerical iteration.
- `InteriorPointSolver` now rejects invalid `sigma` and `stepScale` instead of
  silently clamping user-facing values.
- C++ validation coverage exercises invalid custom parameter structs across the
  affected projection, Newton, and interior-point solvers.
- dartpy LCP tests exercise invalid Python parameter objects routed through the
  solver `parameters` properties.
- Existing Newton warm-start tests were updated so invalid line-search counts
  are no longer used as a control-flow device.
- `CHANGELOG.md` records the new validation behavior.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
           UNIT_math_lcp_math_lcp_lcp_newton_solvers \
           dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='PgsSolverCoverage.RejectsInvalidDivisionEpsilon:ApgdSolverCoverage.RejectsInvalidParameters:TgsSolverCoverage.RejectsInvalidDivisionEpsilon:MinimumMapNewtonCoverage.RejectsInvalidParameters:FischerBurmeisterNewtonCoverage.RejectsInvalidParameters:PenalizedFischerBurmeisterNewtonCoverage.RejectsInvalidParameters:InteriorPointSolverCoverage.RejectsInvalidParameters'
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_newton_solvers \
  --gtest_filter='MinimumMapNewtonSolver.RejectsInvalidLineSearchStepCount:MinimumMapNewtonSolver.GradientDescentWarmStartReducesMerit:MinimumMapNewtonSolver.PgsWarmStartReducesMerit:FischerBurmeisterNewtonSolver.RejectsInvalidLineSearchStepCount:FischerBurmeisterNewtonSolver.GradientDescentWarmStartReducesMerit:FischerBurmeisterNewtonSolver.PgsWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.RejectsInvalidLineSearchStepCount:PenalizedFischerBurmeisterNewtonSolver.GradientDescentWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.PgsWarmStartReducesMerit:PenalizedFischerBurmeisterNewtonSolver.RejectsInvalidLambda'
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
pixi run lint
```

Observed results:

- Focused C++ validation executable: 7/7 tests passed.
- Focused C++ Newton executable: 10/10 tests passed.
- `python/tests/unit/math/test_lcp.py`: 77 passed.
- `pixi run test-lcpsolver`: 17/17 C++ LCP tests passed.
- `pixi run lint` passed, including the LCP solver roster gate.

Immediate next step after this checkpoint:

- Continue the broad objective with the next concrete DART 7 LCP gap, likely a
  solver/interface audit pass that looks for remaining silent fallbacks,
  incomplete support claims, or demo/benchmark coverage gaps beyond parameter
  validation.

## 2026-06-12 Stop-Only Hand-Off — Remaining Parameter Validation Audit

The user explicitly stopped further implementation and requested only hand-off
docs, with no further verification. After that instruction, the only intended
edits are this file and `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`;
do not run lint, build, tests, benchmark listing, solver execution, commit, or
push as part of this stop-only hand-off.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `e3535141c5d Show all LCP solver parameters in py demo`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 22]`.
- Worktree: clean before this docs-only hand-off edit.
- Stash list: empty.
- No PR was associated with this branch when checked earlier in the session.

Recent completed checkpoints on this branch:

- `e3535141c5d Show all LCP solver parameters in py demo`
- `b60e20a8dc8 Expose LCP solver parameters in dartpy`
- `748cef40ea0 Validate advanced LCP solver parameters`
- `06ac009b27c Show LCP solver parameters in py demo`
- `8cd98e2e553 Expose advanced LCP parameters in dartpy`

Current interrupted slice:

- No implementation files were edited in this slice before the stop request.
- The work was in audit/planning only: identify remaining solve-time validation
  gaps in parameterized LCP solvers now that their C++/dartpy parameter
  surfaces are visible.
- Do not treat this as a completed checkpoint. This is a stop-only hand-off
  snapshot.

Remaining audit findings to resume from, if the user explicitly resumes
implementation:

- `PgsSolver`: validate `epsilonForDivision` before iteration; reject
  non-finite or non-positive values.
- `ApgdSolver`: validate `epsilonForDivision > 0` and
  `restartCheckInterval >= 0`. A value of `0` should remain valid because it
  means the restart check runs every iteration.
- `TgsSolver`: validate `epsilonForDivision` before iteration; reject
  non-finite or non-positive values.
- `MinimumMapNewtonSolver`: validate line-search and warm-start parameters
  before iteration, including positive step counts, finite factors in their
  documented ranges, non-negative warm-start iteration counts, and valid PGS
  warm-start relaxation.
- `FischerBurmeisterNewtonSolver`: same Newton line-search and warm-start
  validation as minimum-map Newton, plus positive finite `smoothingEpsilon`.
- `PenalizedFischerBurmeisterNewtonSolver`: keep the existing `lambda` range
  validation and add the same Newton line-search, warm-start, and
  `smoothingEpsilon` validation.
- `InteriorPointSolver`: replace silent clamping of user-facing `sigma` and
  `stepScale` with explicit validation; suggested ranges are finite
  `sigma` in `(0, 1)` and finite `stepScale` in `(0, 1]`.

Relevant cautions for the next session:

- Existing Newton solver tests may intentionally set invalid line-search
  counts, such as `maxLineSearchSteps = 0`, to force failure paths. If
  validation is added, update those expectations to `InvalidProblem` instead
  of preserving numerical-iteration failure behavior.
- Existing validation tests include interior-point cases with `sigma = 1.5`
  and `stepScale = 1.5`; those likely reflect the current silent-clamping
  behavior and should be updated if explicit validation is implemented.
- Existing APGD coverage uses `restartCheckInterval = 0`; keep that valid.

Immediate next step for a fresh session:

1. Do nothing unless the user explicitly resumes implementation.
2. If resumed, confirm current state with `git status --short --branch` and
   `git log --oneline --decorate --max-count=15`.
3. Inspect the solver files listed in `RESUME.md` and implement one bounded
   validation slice with focused C++ and dartpy coverage.

## 2026-06-11 Current Continuation — Full Parameter Metadata In Py-Demo

After checkpoint `b60e20a8dc8 Expose LCP solver parameters in dartpy`, the
Python LCP demo's advanced-parameter table still listed only ADMM, SAP, and
boxed semi-smooth Newton. This slice updates that demo metadata to reflect the
full dartpy-exposed tuning surface.

Current slice:

- Generated the Python LCP demo parameter table from dartpy parameter objects
  for projection, Newton, MPRGP, interior-point, shock propagation, ADMM, SAP,
  and boxed semi-smooth Newton solvers.
- Kept each row linked to a representative benchmark filter when one exists in
  the LCP comparison benchmark metadata.
- Updated py-demo panel tests and `CHANGELOG.md`.

Verification completed for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: `43 passed`.
- `pixi run lint` passed, including the LCP solver roster gate.

## 2026-06-11 Current Continuation — Full dartpy Parameter Surface

After checkpoint `748cef40ea0 Validate advanced LCP solver parameters`, the
next interface-alignment gap is the Python surface for solver-specific tuning.
C++ already exposes `setParameters()` / `getParameters()` on many projection,
Newton, and other LCP solvers, but dartpy only exposed ADMM, SAP, and boxed
semi-smooth Newton.

Current slice:

- Added dartpy parameter classes and `parameters` properties for the remaining
  parameterized LCP solvers: PGS, symmetric PSOR, Jacobi, red-black
  Gauss-Seidel, blocked Jacobi, BGS, NNCG, subspace minimization, APGD, TGS,
  minimum-map Newton, Fischer-Burmeister Newton, penalized
  Fischer-Burmeister Newton, interior-point, MPRGP, and shock propagation.
- Kept Python names `snake_case`, including `lambda_` for the penalized
  Fischer-Burmeister penalty field.
- Updated manual dartpy stubs and Python tests so each exposed parameter object
  can round-trip through the solver `parameters` property.
- Recorded the surface expansion in `CHANGELOG.md`.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Observed results:

- `pixi run build-py-dev` rebuilt and linked `dartpy`.
- `python/tests/unit/math/test_lcp.py`: `70 passed`.
- `pixi run lint` passed, including the LCP solver roster gate.

## 2026-06-11 Current Continuation — Advanced Parameter Validation

The user resumed the broad LCP solver/interface/demo goal after the stop-only
hand-off. This continuation finishes the interrupted advanced solver parameter
validation slice on `feature/lcp-solver-interface-demos`.

Current slice:

- ADMM now rejects invalid advanced parameters before numerical iteration:
  non-positive or non-finite `rhoInit`, negative or non-finite `muProx`, and
  `adaptiveRhoTolerance <= 1`.
- SAP now rejects invalid advanced parameters before numerical iteration:
  non-positive or non-finite `regularization`, Armijo and backtracking factors
  outside `(0, 1)`, and non-positive line-search iteration counts.
- Boxed semi-smooth Newton now rejects invalid advanced line-search and
  regularization parameters before numerical iteration:
  non-positive line-search step counts, `stepReduction` outside `(0, 1)`,
  `sufficientDecrease` outside `[0, 1)`, non-positive `minStep`, and negative
  `jacobianRegularization`.
- C++ smoke and validation tests cover invalid custom option structs for the
  affected solvers.
- dartpy LCP tests cover invalid Python parameter objects routed through the
  solver `parameters` properties.
- `CHANGELOG.md` records the new solve-time validation behavior.

Verification completed for this slice:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  --gtest_filter='BoxedSemiSmoothNewtonSolverCoverage.*:AdmmSolverCoverage.*'
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
```

Observed results:

- The explicit validation target rebuilt and relinked, resolving the stale-test
  symptom from the interrupted hand-off.
- Focused boxed semi-smooth Newton and ADMM validation coverage passed 4 tests.
- C++ LCP suite: `100% tests passed, 0 tests failed out of 17`.
- Focused dartpy LCP tests: `69 passed`.

Remaining before checkpoint commit:

- Run `pixi run lint` and include any formatter/docs updates in the checkpoint.

## 2026-06-11 Critical Stop-Only Hand-Off — Advanced Parameter Validation

The user explicitly stopped further work and requested only hand-off docs, with
no further verification. After that instruction, the only intended edits are
this file and `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`; no lint,
build, tests, benchmark listing, solver execution, commit, or push should be
run as part of this stop-only hand-off.

Repository state observed before this docs-only hand-off edit:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `06ac009b27c Show LCP solver parameters in py demo`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 19]`.
- Recent local checkpoints:
  - `06ac009b27c Show LCP solver parameters in py demo`
  - `8cd98e2e553 Expose advanced LCP parameters in dartpy`
  - `17a994e3772 Reject negative LCP friction coefficients`
  - `e3353bf04b7 Expose LCP solver sweep metadata`
- Uncommitted implementation files observed before this docs-only hand-off:
  - `CHANGELOG.md`
  - `dart/math/lcp/newton/boxed_semi_smooth_newton_solver.cpp`
  - `dart/math/lcp/other/admm_solver.cpp`
  - `dart/math/lcp/other/sap_solver.cpp`
  - `python/tests/unit/math/test_lcp.py`
  - `tests/unit/math/lcp/test_all_solvers_smoke.cpp`
  - `tests/unit/math/lcp/test_lcp_validation_and_solvers.cpp`

Current uncommitted slice:

- Adds solve-time validation for advanced ADMM parameters:
  `rhoInit > 0`, `muProx >= 0`, and `adaptiveRhoTolerance > 1`.
- Adds solve-time validation for advanced SAP parameters:
  positive `regularization`, `armijosParameter` and `backtrackingFactor` in
  `(0, 1)`, and positive `maxLineSearchIterations`.
- Adds solve-time validation for boxed semi-smooth Newton parameters:
  positive `maxLineSearchSteps`, `stepReduction` in `(0, 1)`,
  `sufficientDecrease` in `[0, 1)`, positive `minStep`, and non-negative
  `jacobianRegularization`.
- Returns `InvalidProblem` with parameter-specific messages before numerical
  iteration begins.
- Adds focused C++ coverage in the all-solvers smoke test and validation
  coverage, plus dartpy coverage for invalid advanced parameter values.
- Updates `CHANGELOG.md` for the new validation behavior.

Verification that had already run before the stop-only instruction:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy \
  --parallel "$JOBS"
build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_all_solvers_smoke
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run test-lcpsolver
```

Observed results before the stop-only instruction:

- After a const-qualification fix in
  `tests/unit/math/lcp/test_all_solvers_smoke.cpp`, the explicit build of
  `UNIT_math_lcp_math_lcp_all_solvers_smoke` and `dartpy` succeeded.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed 17 tests.
- `python/tests/unit/math/test_lcp.py` passed 69 tests.
- `pixi run test-lcpsolver` still failed only
  `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers`. The failure output still
  appeared to reference the older
  `BoxedSemiSmoothNewtonSolverCoverage.FailureAndValidationBranches` test name
  and old expectations, even after the source had been edited, so the next
  session should suspect a stale or unrelinked test executable before assuming
  the source patch is wrong.

Immediate next step for a fresh session:

1. Confirm current state with `git status --short --branch` and recent log.
2. Inspect the uncommitted implementation and these hand-off docs.
3. If implementation work is explicitly resumed, first rebuild only
   `UNIT_math_lcp_math_lcp_lcp_validation_and_solvers` and rerun that focused
   executable before rerunning broader LCP verification.

Do not treat this as a completed checkpoint. This is an intentionally
uncommitted, unpushed hand-off snapshot.

That interrupted state is historical after the current continuation.

## 2026-06-11 Current Continuation — Advanced Solver Parameters

The user later explicitly resumed the broad LCP solver/interface/demo goal, so
the previous stop-only hand-off is historical. This continuation resumes from
`17a994e3772 Reject negative LCP friction coefficients` on
`feature/lcp-solver-interface-demos` and adds a bounded dartpy interface slice.

Current slice:

- dartpy now exposes `AdmmSolverParameters`, `SapSolverParameters`, and
  `BoxedSemiSmoothNewtonSolverParameters` with DART 7 snake_case field names.
- dartpy `AdmmSolver`, `SapSolver`, and `BoxedSemiSmoothNewtonSolver` now have
  `parameters` properties backed by the existing C++ `setParameters()` /
  `getParameters()` APIs.
- Python LCP tests cover default values, round-tripping customized settings,
  and solving a small boxed problem with customized advanced solver parameters.
- The Python LCP demo now reports the advanced solver parameter names, default
  values, and matching benchmark sweep filters in `setup.info` and the GUI
  panel.
- The manual dartpy stubs, generated API boundary inventory, LCP roster lint
  guard, and changelog describe the new Python-facing surface.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run build-py-dev
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
pixi run lint
```

Observed results:

- `pixi run build-py-dev` rebuilt and linked `dartpy`.
- `python/tests/unit/math/test_lcp.py`: `66 passed`.
- `python/tests/unit/test_py_demo_panels.py`: `43 passed`.
- LCP solver roster check: `24 solvers, 24 standard, 16 boxed/findex`.
- `pixi run lint` passed.

Current observed state before this slice:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD:
  `17a994e3772 Reject negative LCP friction coefficients`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 17]`.
- Recent local checkpoints:
  - `17a994e3772 Reject negative LCP friction coefficients`
  - `e3353bf04b7 Expose LCP solver sweep metadata`
  - `46700198d80 Expose LCP scale benchmark metadata`
  - `197b55c335a Use concrete LCP contact registration gates`
- No PR was associated with this branch when checked before the stop-only
  hand-off.

A fresh session should read this file and `RESUME.md`, inspect branch state,
and continue from the verification/commit state recorded here. The broad LCP
solver/interface/demo objective remains open; this dev-task folder must not be
retired from this checkpoint.

## 2026-06-11 Latest Completed Implementation — Friction Coefficient Validation

The current continuation resumes from
`e3353bf04b7 Expose LCP solver sweep metadata` on
`feature/lcp-solver-interface-demos`. The two docs-only stop-hand-off edits
from the previous instruction are preserved in the working tree and this
continuation adds a bounded DART 7 LCP problem-interface validation slice.

Current slice:

- Shared LCP problem validation now rejects friction-index rows whose stored
  upper coefficient `hi[i]` is negative. The effective-bound helper now uses
  that non-negative coefficient directly instead of accepting a negative value
  and normalizing it with `abs()`.
- `LcpProblem::hasFrictionIndex()` now treats negative friction coefficients as
  invalid problem metadata, matching `LcpProblem::isValid()` and solver support
  checks.
- C++ coverage exercises public problem classification, raw validation,
  effective-bound construction, and generated invalid-problem solver rejection.
- dartpy coverage exercises the same invalid problem through
  `LcpProblem`, `supports_problem()`, and `DantzigSolver.solve()`.

Verification completed for this slice:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
```

Observed results:

- C++ LCP suite: `100% tests passed, 0 tests failed out of 17`.
- The focused dartpy LCP test reported `62 passed`.
- `pixi run lint` passed before commit.

## 2026-06-11 Historical Stop-Only Hand-Off

The previous user instruction was to stop all implementation work, do not run
further verification, and ensure only the hand-off docs were current. After
that instruction, the only edits were this file and
`docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.

No lint, build, tests, benchmark listing, benchmark execution, solver
execution, code edits, commit, or push was run after that stop-only
instruction. The last observed repository state before that docs-only hand-off
edit was:

- Branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `e3353bf04b7 Expose LCP solver sweep metadata`.
- Tracking state:
  `feature/lcp-solver-interface-demos...origin/feature/lcp-solver-interface-demos [ahead 16]`.
- Recent local checkpoints:
  - `e3353bf04b7 Expose LCP solver sweep metadata`
  - `46700198d80 Expose LCP scale benchmark metadata`
  - `197b55c335a Use concrete LCP contact registration gates`
- The worktree was clean before this final docs-only hand-off edit.

Do not continue implementation from this hand-off alone. A fresh session should
first inspect `git status --short --branch` and `git log --oneline --decorate
--max-count=15`, read this file and `RESUME.md`, then wait for or obtain an
explicit next task before changing code. The broad LCP solver/interface/demo
objective remains open; this dev-task folder must not be retired from this
stop-only checkpoint.

That stop-only state is historical after the current continuation.

## Current Status

- [x] Consolidated the current work on one branch:
      `feature/lcp-solver-interface-demos`.
- [x] Merged the latest `origin/main` into the branch before publishing the
      active checkpoint stack.
- [x] Added representative LCP demo/benchmark packets for active
      friction-index contact cases.
- [x] Refined Dantzig friction-index solves so tangent bounds are re-solved
      after final normal-scaled friction bounds shift.
- [x] Exposed shared LCP problem validation diagnostics through C++ and dartpy:
      `LcpProblem::isValid()`, `LcpProblem::getValidationMessage()`,
      `LcpProblem.is_valid()`, and `LcpProblem.get_validation_message()`.
- [x] Tightened per-problem native support reporting so `DirectSolver` no
      longer reports large standard packets as native when it will delegate to
      Dantzig.
- [x] Captured the current hand-off state after the user explicitly requested
      no further verification or implementation work.
- [x] Exposed the DART 7 contact-pipeline comparison sweeps in py-demo LCP
      benchmark metadata.
- [x] Added a live billiard symmetry-error metric alongside momentum and energy
      error in the LCP py-demo.
- [x] Pointed the high-mass-ratio stack demo metadata at solver-manifest stack
      contact benchmarks as well as the boxed world-step benchmark.
- [x] Added GUI plots for billiard momentum, energy, and symmetry invariant
      histories in the LCP py-demo.
- [x] Added a representative benchmark-suite filter and command to the LCP
      py-demo metadata, derived from the benchmark packet table while keeping
      the smoke command intact.
- [x] Realigned LCP background docs with DART 7 snake_case solver header paths
      and extended the LCP roster lint gate to catch stale documented paths.
- [x] Captured the critical stop-and-handoff state without running further
      verification, including the interrupted MPRGP support-predicate audit.
- [x] Tightened MPRGP per-problem native support reporting so non-symmetric and
      non-positive-definite standard packets are marked delegated instead of
      native.
- [x] Tightened Baraff per-problem native support reporting so non-symmetric and
      indefinite standard packets delegate to Dantzig instead of entering the
      native active-set loop.
- [x] Refreshed the branch against `origin/main`, captured the final hand-off
      state, and stopped without running any further verification.
- [x] Filtered singular-degenerate standard benchmark registrations through
      concrete `supportsProblem(problem)` checks so MPRGP fallback rows are not
      listed as native while Baraff PSD rows remain.
- [x] Filtered contact-normal standard benchmark registrations through
      concrete normal-only contact packets, preserving MPRGP/Baraff native rows
      and replacing Direct's size special case with concrete support checks.
- [x] Captured the 2026-06-11 critical hand-off after the contact-normal
      checkpoint with no further lint, tests, benchmark listing, solver
      execution, or implementation work.
- [x] Added manifest-driven active friction-index contact benchmark rows to
      `lcp_compare` and retargeted the py-demo representative metadata from the
      older two-solver benchmark surface to that main comparison filter.
- [x] Filtered active-set transition benchmark registrations through concrete
      `supportsProblem(problem)` checks, replacing the Direct-only standard
      packet special case with the shared native-route gate.
- [x] Filtered pivoting scale sweep benchmark registrations through concrete
      generated problem support and removed the now-unused manifest-family
      helper from `lcp_compare`.
- [x] Captured the final 2026-06-11 consolidated hand-off after the latest
      two local benchmark-routing commits. No lint, build, tests, benchmark
      listing, solver execution, or further implementation work was run after
      the user's critical stop instruction.
- [x] Filtered the manifest-generated `BM_LcpCompare` and serial/parallel
      `BM_LcpBatch` argument rows through concrete generated-problem support,
      keeping benchmark problem sizes aligned with solver native-route
      predicates.
- [x] Filtered separated world-contact, world stack-contact, and contact-solver
      comparison benchmark rows through concrete generated contact packets
      while avoiding registration-time generation of the largest dense
      box/articulated fixtures.
- [x] Captured the critical no-verification hand-off after local implementation
      commit `8f0242c2442` so a fresh session can resume from the consolidated
      branch without relying on chat context.
- [x] Added concrete support gates to the remaining heavyweight contact
      benchmark registrations: exact generated-batch checks for mixed
      world-contact batches and representative small contact probes for dense
      box-contact, articulated unified-contact, and dense box-contact batch rows.
- [x] Aligned generated LCP correctness coverage with concrete
      `supportsProblem(problem)` predicates instead of manifest-family support.
- [x] Captured the final no-verification hand-off after the latest two local
      implementation commits, refreshed against `main`, so a fresh session can
      resume from the single consolidated branch.
- [x] Filtered grouped serial/parallel batch benchmark registrations through
      concrete generated grouped-batch support for their exact published
      variants.
- [x] Removed residual manifest-family prechecks from `lcp_compare`
      registration paths that already use concrete generated problem filters.
- [x] Updated the Python LCP demo solver profile so native coverage is derived
      from concrete representative problem cases instead of only static
      manifest surfaces.
- [x] Captured the latest critical no-verification hand-off after the Python
      demo concrete native-case profile slice, with the branch refreshed
      against `main` and ready for publication as one consolidated branch.
- [x] Filtered larger, stress, extreme, and production active-set transition
      benchmark registrations through concrete generated-problem support,
      including exact production-batch problem-list checks.
- [x] Filtered mildly ill-conditioned and near-singular benchmark
      registrations through concrete generated-problem support, including exact
      serial/parallel batch problem-list checks.
- [x] Filtered singular-degenerate friction-index and standard/boxed batch
      benchmark registrations through exact generated batch support.
- [x] Captured the latest 2026-06-11 critical hand-off after the conditioning
      benchmark-routing checkpoint, refreshed the branch against current
      `main`, and stopped without running further verification.
- [x] Routed the all-solvers LCP smoke-test skip helper through concrete
      `supportsProblem(problem)` predicates and verified the focused LCP test
      suite.
- [x] Removed redundant manifest-family prechecks from concrete
      benchmark-routing helpers for active-set transition, mildly
      ill-conditioned, near-singular, and singular-degenerate packets.
- [x] Removed redundant manifest-family prechecks from contact benchmark
      registration paths that already use concrete contact support probes.
- [x] Captured the final stop-only hand-off at local checkpoint
      `197b55c335a`, with no lint, build, tests, benchmark listing, solver
      execution, push, or implementation work after the user's final stop
      instruction.
- [x] Extended the LCP py-demo representative benchmark metadata to expose
      active-set scale/production rows and larger/stress/extreme
      singular-degenerate rows in the generated representative command.
- [x] Exposed solver-specific tuning and robustness sweep benchmarks through
      the LCP py-demo representative metadata.
- [x] Captured the final stop-only hand-off at local checkpoint
      `e3353bf04b7`, with no lint, build, tests, benchmark listing, solver
      execution, commit, push, code edits, or further implementation work after
      the user's final stop-only instruction.
- [x] Reject negative friction-index `hi` coefficients consistently across C++
      validation, effective-bound construction, generated invalid-problem
      coverage, and dartpy.
- [x] Expose advanced boxed/friction-index solver parameter objects and
      `parameters` properties in dartpy, with focused Python tests and lint.
- [x] Add the advanced solver parameter summary to the Python LCP demo panel
      and metadata, with focused py-demo panel tests.
- [x] Validate advanced ADMM, SAP, and boxed semi-smooth Newton parameter
      structs before numerical iteration and cover the behavior from C++ and
      dartpy.
- [x] Expose the remaining parameterized C++ LCP solver tuning structs through
      dartpy with matching stubs and Python round-trip tests.
- [x] Extend the Python LCP demo's advanced-parameter metadata from the three
      initially exposed advanced solvers to the full dartpy-exposed tuning
      surface.
- [ ] Continue the remaining DART 7 audit of LCP solver/problem interfaces and
      py-demo coverage from a fresh session.

## Goal

Keep the DART 7 standalone LCP surface coherent across C++, dartpy, tests,
benchmarks, and py-demos. A fresh session should be able to compare solver
families on standard, boxed, and friction-index packets, understand solver
capability limits, and extend representative challenging examples without
rediscovering the current branch state.

## Non-Goals

- Do not rebase or split the published branch unless the maintainer explicitly
  asks for history surgery.
- Do not treat the broad LCP goal as complete just because the validation API
  checkpoint is pushed.
- Do not retire this dev-task folder until remaining follow-up work is either
  completed or moved to durable planning/design docs.

## Key Decisions

- Use one additive feature branch,
  `feature/lcp-solver-interface-demos`, for the current LCP interface/demo
  continuation.
- Keep `detail::validateProblem(const LcpProblem&)` routed through the public
  `LcpProblem` validation diagnostic so C++ solvers, C++ tests, and Python
  demos share the same first-failure message.
- Treat `LcpSolver::supportsProblem(problem)` as per-problem native support,
  not only surface-family support. Solver-specific native limits should be
  reflected there when the public `solve()` path delegates internally.
- Leave the raw matrix/vector validation overload in `lcp_validation.hpp` in
  place for low-level call sites that validate temporary raw data without
  constructing another `LcpProblem`.
- Treat the active-bound two-contact friction-index packet as a regression and
  demo benchmark seed for Dantzig and friction-index-capable iterative solvers.

## Latest Code Checkpoint

The current committed implementation checkpoint is the dartpy advanced
solver-parameter slice. It exposes Python parameter objects and solver
`parameters` properties for ADMM, SAP, and boxed semi-smooth Newton, then
surfaces those tunable knobs in the Python LCP demo next to the matching
benchmark sweeps.

The current uncommitted implementation slice adds solve-time validation for
those advanced solver parameter structs. It is intentionally left uncommitted
and unpushed in this hand-off.

The next checkpoint after that hand-off completes the validation slice and
verifies it with focused C++/Python LCP tests. The broader LCP
solver/interface/demo objective remains open.

## Py-Demo Representative Scale Metadata Checkpoint

The latest Python-demo metadata checkpoint adds representative benchmark packet
rows for scale and solver-tuning coverage:

- `active_set_scale` points at `BM_LcpLargerActiveSetTransition`,
  `BM_LcpStressActiveSetTransition`, `BM_LcpExtremeActiveSetTransition`,
  `BM_LcpProductionActiveSetTransition`, and the serial/parallel production
  active-set transition batch rows.
- `singular_degenerate_scale` points at the larger/stress/extreme
  singular-degenerate rows and the serial/parallel singular-degenerate batch
  rows.
- `solver_parameter_sweeps` points at solver-specific relaxation, line-search,
  pivoting-scale, block-partition, restart-policy, iteration-budget,
  shock-layer, SPD-check, path-following, ADMM rho, and SAP regularization
  benchmark sweeps.

This keeps `representative_benchmark_filter` and
`representative_benchmark_command` aligned with the current hard-case and
scalability benchmark registrations instead of only the smaller transition and
base singular-degenerate names.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(Larger|Stress|Extreme|Production)ActiveSetTransition|BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)|BM_Lcp(Larger|Stress|Extreme)SingularDegenerate|BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(PgsRelaxationSweep|SymmetricPsorRelaxationSweep|RedBlackGaussSeidelRelaxationSweep|BoxedSemiSmoothNewtonLineSearchSweep|PivotingScaleSweep|BlockPartitionSweep|ApgdRestartSweep|TgsIterationBudgetSweep|NncgPgsIterationsSweep|SubspaceMinimizationPgsIterationsSweep|ShockPropagationLayerSweep|MprgpSpdCheckSweep|InteriorPointPathSweep|AdmmRhoSweep|SapRegularizationSweep)'
pixi run lint
```

Observed results:

- The Python demo-panel test reported `43 passed`.
- The benchmark-list check rebuilt/linked `BM_LCP_COMPARE` and listed the
  newly exposed active-set scale/production and singular-degenerate
  scale/batch benchmark rows.
- A second benchmark-list check listed the newly exposed solver-specific
  tuning and robustness sweep rows.
- `pixi run lint` passed.

## Contact Benchmark Registration Cleanup Checkpoint

The latest implementation checkpoint removes redundant manifest-family
prechecks from contact benchmark registration paths:

- `RegisterActiveFrictionIndexContactBenchmarks()`,
  `RegisterWorldContactBenchmarks()`, `RegisterWorldBoxContactBenchmarks()`,
  `RegisterWorldStackContactBenchmarks()`,
  `RegisterArticulatedUnifiedContactBenchmarks()`,
  `RegisterContactSolverComparisonSweepBenchmarks()`,
  `RegisterContactNormalStandardSweepBenchmarks()`,
  `RegisterWorldContactBatchBenchmarks()`, and
  `RegisterWorldBoxContactBatchBenchmarks()` now rely on their concrete
  generated contact packets or support probes.
- Dense world-box contact registrations still keep their explicit
  `SupportsDenseWorldBoxContactPatch(...)` solver scope before the concrete
  probe, avoiding unrelated solver/performance expansion.
- The representative LCP benchmark/test/demo surface now has no remaining
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::...)` prechecks
  in these audited paths.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(ActiveFrictionIndexContact|WorldContact|WorldBoxContact|WorldStackContact|ArticulatedUnifiedContact|ContactSolverComparisonSweep|ContactNormalStandardSweep|WorldContactBatchSerial|WorldContactStressBatchSerial|WorldContactPipeline32BatchSerial|WorldBoxContactBatchSerial).*(Dantzig|Pgs|MPRGP|Baraff|BoxedSemiSmoothNewton)'
pixi run lint
```

Observed results:

- The benchmark-list check rebuilt and linked `BM_LCP_COMPARE`.
- It listed representative concrete rows for active friction-index contact,
  world contact, dense world-box contact, world stack contact, articulated
  unified contact, contact solver comparison, normal-standard contact, mixed
  world-contact batches, and dense world-box contact batches.
- `pixi run lint` passed, including the LCP solver roster check.

## Concrete Benchmark Helper Cleanup Checkpoint

The latest implementation checkpoint removes redundant manifest-family gates
from helper predicates that already have concrete generated problems:

- `SolverShouldRunMildIllConditionedBenchmark(...)`,
  `SolverShouldRunNearSingularBenchmark(...)`,
  `SolverShouldRunLargerActiveSetTransitionBenchmark(...)`, and
  `SolverShouldRunSingularDegenerateBenchmark(...)` now rely on their explicit
  solver scopes plus `SolverSupportsConcreteProblem(...)`.
- The now-unused
  `getMildIllConditionedProblemSupport(...)`,
  `getNearSingularProblemSupport(...)`,
  `getLargerActiveSetTransitionProblemSupport(...)`, and
  `getSingularDegenerateProblemSupport(...)` helpers were removed.
- The production active-set transition batch helper now shares the simpler
  concrete-problem helper signature.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(MildIllConditioned|NearSingular|LargerActiveSetTransition|ProductionActiveSetTransitionBatchSerial|SingularDegenerate|SingularDegenerateFrictionIndexBatchSerial|SingularDegenerateStandardBoxedBatchSerial)/(Standard32|Boxed16|FrictionIndex8|Standard8|Boxed8|CoupledFrictionIndex3|Standard16|CoupledFrictionIndex6|CoupledFrictionIndex8)/(Direct|MPRGP|Baraff|Dantzig|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run lint
```

Observed results:

- The first benchmark-list attempt caught an unused `testCase` parameter after
  the precheck removal; the helper signature and callers were simplified.
- The rerun rebuilt and linked `BM_LCP_COMPARE`, then listed concrete active
  set, production batch, mild/near-singular, and singular-degenerate rows for
  the scoped solver/problem combinations.
- `pixi run lint` passed, including the LCP solver roster check.

## All-Solvers Smoke Test Support-Routing Checkpoint

The latest implementation checkpoint aligns all-solvers smoke-test skip logic
with concrete solver support:

- `tests/unit/math/lcp/test_all_solvers_smoke.cpp` no longer maps
  `ProblemCategory` to manifest-family `LcpProblemSupport` for the local
  `canSolve(...)` helper.
- `canSolve(...)` now constructs the solver and asks its concrete
  `supportsProblem(problem.problem)` predicate whether the exact factory
  problem is native-supported.
- This keeps generated smoke coverage aligned with solver-specific native-route
  predicates such as Direct's small standard-LCP window and MPRGP/Baraff
  matrix-domain limits.

Verification for this checkpoint:

```bash
pixi run test-lcpsolver
pixi run lint
```

Observed results:

- The focused LCP test task rebuilt as needed and reported
  `100% tests passed, 0 tests failed out of 17`.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke` passed.
- `pixi run lint` passed, including the LCP solver roster check.

## Pivoting Scale Sweep Support-Routing Checkpoint

The latest implementation checkpoint aligns pivoting scale sweep registration
with concrete solver support:

- `BM_LcpPivotingScaleSweep` registration now builds each exact generated
  problem once and publishes a row only when the selected solver's concrete
  `supportsProblem(problem)` predicate accepts it.
- `RunPivotingScaleSweepBenchmark` now uses the same concrete guard before
  execution, so future additions cannot silently run fallback/delegated rows as
  native pivoting comparisons.
- The now-unused manifest-family `getProblemSupport(...)` helper was removed
  from `lcp_compare`.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpPivotingScaleSweep/(Standard|Boxed|FrictionIndex)/(Direct|Lemke|Baraff|Dantzig)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpPivotingScaleSweep/Standard/Direct/Rows3|BM_LcpPivotingScaleSweep/Standard/Baraff/Rows8|BM_LcpPivotingScaleSweep/Boxed/Dantzig/Rows12|BM_LcpPivotingScaleSweep/FrictionIndex/Dantzig/Contacts4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The first benchmark-list attempt caught that `getProblemSupport(...)` had
  become unused under `-Werror`; the helper was removed.
- The rerun rebuilt `BM_LCP_COMPARE` and listed the expected concrete pivoting
  rows: Direct standard rows 2 and 3, Lemke/Baraff/Dantzig standard rows,
  Dantzig boxed rows, and Dantzig friction-index rows.
- The short benchmark execution reported `contract_ok=1` for sampled Direct
  standard, Baraff standard, Dantzig boxed, and Dantzig friction-index rows.

## Singular-Degenerate Batch Support-Routing Checkpoint

The latest implementation checkpoint aligns singular-degenerate batch
registration with concrete solver support:

- Singular-degenerate friction-index serial/parallel batch rows now build the
  exact four-problem generated batch used by each published row and require
  every problem to pass the solver's concrete `supportsProblem(problem)` route.
- Singular-degenerate standard/boxed serial/parallel batch rows now use the same
  exact-batch check instead of relying only on the single-problem helper.
- Single singular-degenerate registration now precomputes the generated problem
  once per case and passes it through the shared concrete helper.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpSingularDegenerate(FrictionIndexBatch|StandardBoxedBatch)(Serial|Parallel)/(Standard16|Boxed16|CoupledFrictionIndex6|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex256)/(Direct|MPRGP|Baraff|Admm|Sap|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpSingularDegenerateFrictionIndexBatchSerial/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateFrictionIndexBatchParallel/CoupledFrictionIndex6/Sap|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Standard16/Baraff|BM_LcpSingularDegenerateStandardBoxedBatchSerial/Boxed16/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The benchmark-list check rebuilt `BM_LCP_COMPARE` and listed exact supported
  serial/parallel singular-degenerate batch rows for Admm, Sap, and
  BoxedSemiSmoothNewton on coupled friction-index batches; Baraff, Admm, and
  Sap on standard batches; and Admm, Sap, and BoxedSemiSmoothNewton on boxed
  batches. Direct and MPRGP rows were absent from the scoped filter.
- The short benchmark execution reported `contract_ok=1` for sampled Sap
  friction-index serial and parallel batch rows, Baraff standard-batch rows,
  and BoxedSemiSmoothNewton boxed-batch rows.

## 2026-06-11 Latest Critical Hand-Off Snapshot

The latest user instruction was to stop implementation work and focus on
hand-off only, with no further verification. After that instruction, no lint,
build, tests, benchmark-list commands, benchmark execution, or solver execution
were run. The only remaining work in this checkpoint is repository
housekeeping: record this hand-off state and publish the consolidated branch.

Branch and base state:

- Consolidated branch: `feature/lcp-solver-interface-demos`.
- Latest implementation commit:
  `9a17ba85aa5 Filter conditioning LCP benchmarks concretely`.
- Current `main` was fetched from `https://github.com/dartsim/dart.git` at
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`.
- This checkout's `origin` remote is configured as SSH. Earlier successful
  publishes used HTTPS, so a fresh session should fetch the branch directly
  from GitHub before trusting stale local remote-tracking metadata.

Fresh-session resume checklist:

1. Fetch and check out `feature/lcp-solver-interface-demos` from GitHub.
2. Read this file and
   `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
3. Inspect the branch tip and choose one bounded remaining LCP interface/demo
   audit gap.
4. Do not mark the broad LCP objective complete or retire this dev-task folder
   from this hand-off alone.

Likely remaining follow-ups:

- Continue auditing any benchmark/demo/test surfaces that still summarize
  solver native support without a concrete `supportsProblem(problem)` check.
- Review remaining solver-domain predicates against actual native solve paths,
  especially where public `solve()` delegates internally.
- Continue py-demo and benchmark apples-to-apples coverage work after the next
  session has re-established local verification.

## Mild/Near-Singular Benchmark Support-Routing Checkpoint

The latest implementation checkpoint aligns conditioning benchmark registration
with concrete solver support:

- `BM_LcpMildIllConditioned` and `BM_LcpNearSingular` registrations now build
  the generated problem for each published case once and keep only solver rows
  whose concrete `supportsProblem(problem)` predicate accepts that packet.
- Mildly ill-conditioned and near-singular serial/parallel batch registrations
  now build the exact four-problem batch used by each row and require every
  concrete batch problem to be supported before publishing the row.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpMildIllConditioned/(Standard32|Boxed16|FrictionIndex8|ExtremeCoupledFrictionIndex256)/(Dantzig|Baraff|MPRGP|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpNearSingular/(Standard8|Boxed8|CoupledFrictionIndex3|CoupledFrictionIndex256)/(Dantzig|Baraff|MPRGP|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(MildIllConditioned|NearSingular)Batch(Serial|Parallel)/(Standard32|Boxed16|FrictionIndex8|CoupledFrictionIndex8|ExtremeCoupledFrictionIndex256|Standard8|Boxed8|CoupledFrictionIndex3)/(Dantzig|Baraff|MPRGP|Pgs|ShockPropagation|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpMildIllConditioned/Standard32/Baraff|BM_LcpMildIllConditionedBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton|BM_LcpNearSingular/Boxed8/BoxedSemiSmoothNewton|BM_LcpNearSingularBatchSerial/CoupledFrictionIndex3/ShockPropagation' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The mild row-list check rebuilt `BM_LCP_COMPARE` and listed scoped concrete
  rows for Standard32, Boxed16, FrictionIndex8, and
  ExtremeCoupledFrictionIndex256; MPRGP and Direct were absent.
- The near-singular row-list check listed scoped concrete rows for Standard8,
  Boxed8, CoupledFrictionIndex3, and CoupledFrictionIndex256; MPRGP, PGS, and
  Direct were absent.
- The batch row-list check listed serial and parallel mild/near-singular rows
  for the exact supported four-problem batches.
- The short benchmark execution reported `contract_ok=1` for the sampled
  mild/near-singular single and serial-batch rows.
- `pixi run lint`: passed.

## Larger Active-Set Transition Benchmark Support-Routing Checkpoint

The latest implementation checkpoint aligns active-set scaling benchmark
registration with concrete solver support:

- Larger, stress, extreme, and production active-set transition benchmark
  registrations now build the generated problem for each published case once
  and keep only solver rows whose concrete `supportsProblem(problem)` predicate
  accepts that packet.
- Production active-set transition batch registrations now build the exact
  four-problem batch used by the benchmark row and require every concrete batch
  problem to be supported before publishing serial or parallel rows.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp(Larger|Stress|Extreme|Production)ActiveSetTransition/(Standard32|Boxed32|CoupledFrictionIndex8|Standard64|Boxed64|CoupledFrictionIndex12|Standard128|Boxed128|CoupledFrictionIndex16|CoupledFrictionIndex24)/(MPRGP|Baraff|Direct|Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpProductionActiveSetTransitionBatch(Serial|Parallel)/(Standard32|Boxed32|CoupledFrictionIndex8)/(MPRGP|Baraff|Direct|Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpLargerActiveSetTransition/Standard32/MPRGP|BM_LcpStressActiveSetTransition/Boxed64/Pgs|BM_LcpProductionActiveSetTransitionBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The single-row benchmark-list check rebuilt `BM_LCP_COMPARE` and listed
  concrete larger/stress/extreme/production active-set transition rows for
  Dantzig, PGS, MPRGP, and BoxedSemiSmoothNewton where the generated packets
  are supported; Direct and Baraff were absent from the filtered row set.
- The production-batch benchmark-list check listed serial and parallel rows for
  Standard32, Boxed32, and CoupledFrictionIndex8 for the supported solver set.
- The short benchmark execution reported `contract_ok=1` for
  `BM_LcpLargerActiveSetTransition/Standard32/MPRGP`,
  `BM_LcpStressActiveSetTransition/Boxed64/Pgs`, and
  `BM_LcpProductionActiveSetTransitionBatchSerial/CoupledFrictionIndex8/BoxedSemiSmoothNewton/4`.
- `pixi run lint`: passed.

## Python Demo Concrete Native-Case Profile Checkpoint

The latest implementation checkpoint improves the LCP py-demo's apples-to-apples
solver profile:

- The representative solver profile now derives its native coverage label from
  the actual concrete representative problem rows instead of the static
  standard/boxed/findex manifest flags.
- The panel column now reports concrete case coverage such as `standard 2/4`
  for Direct and `standard 4/4, boxed 3/3, findex 2/2` for full-surface
  solvers, making partial native support visible without opening the
  per-problem detail table.

Verification already completed before the latest critical hand-off instruction:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

No lint, build, test, benchmark-list, or solver-execution verification was run
after the final stop-and-handoff instruction. The hand-off checkpoint is
intentionally committed without further validation per that instruction.

## 2026-06-11 Critical Consolidated Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only,
with no further verification. This is the latest snapshot a fresh Claude/Codex
session should trust before resuming.

Branch state before the final hand-off checkpoint commit:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local implementation HEAD:
  `2cd337aa0cf Use concrete gates for LCP benchmark registration`.
- Working tree contained the Python demo concrete native-case profile slice in
  `CHANGELOG.md`, `python/examples/demos/scenes/lcp_physics.py`,
  `python/tests/unit/test_py_demo_panels.py`, and these dev-task docs.
- Remote-tracking branch in this checkout still showed
  `origin/feature/lcp-solver-interface-demos` at
  `737b9c95c11 Document final LCP handoff checkpoint`; treat that as stale
  because previous pushes used HTTPS while `origin` is configured for SSH.
- Local first-parent stack ahead of that stale tracking ref:
  - `2cd337aa0cf Use concrete gates for LCP benchmark registration`
  - `6c0763927a5 Filter grouped LCP batch rows concretely`
  - `20aaa23d0fe Document final LCP no-verification handoff`
  - `fe5b70b32cc Route generated LCP coverage by concrete support`
  - `f86e353df2f Gate heavyweight LCP contact benchmarks concretely`
- `main` was fetched over HTTPS from `https://github.com/dartsim/dart.git` to
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`

This final checkpoint should be pushed to the same consolidated branch,
`feature/lcp-solver-interface-demos`. The broad LCP objective is still open;
do not retire this dev-task folder or mark the work complete from this
checkpoint alone.

Recommended resume entrypoint:

1. Fetch `feature/lcp-solver-interface-demos` directly from GitHub and inspect
   the branch tip; do not rely on the stale SSH tracking ref recorded above.
2. Read this file and
   `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
3. Continue one bounded LCP interface/demo audit gap at a time, starting with
   remaining demos/tests that summarize native rows without a concrete
   `supportsProblem(problem)` check.

## Benchmark Concrete-Gate Cleanup Checkpoint

The latest implementation checkpoint removes the last coarse manifest-family
prechecks from benchmark registration paths that already had concrete generated
problem filters:

- `BM_LcpCompare`, `BM_LcpActiveSetTransition`, `BM_LcpBatchSerial`, and
  `BM_LcpBatchParallel` now rely on concrete generated packet support checks to
  decide whether rows should be published.
- `bm_lcp_compare.cpp` no longer contains registration-time checks of the form
  `supportsProblem(solver, getProblemSupport(...))`; the remaining benchmark
  support gates instantiate solvers and call `supportsProblem(problem)` on the
  generated packet or problem list.

Verification for this checkpoint:

```bash
rg -n "supportsProblem\\(solver, getProblemSupport|supportsProblem\\(solverEntry|supportsProblem\\(solver,|supportsProblem\\(\\*solverEntry" \
  tests/benchmark/lcpsolver/bm_lcp_compare.cpp
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpCompare/Standard/(Direct|MPRGP|Baraff)|BM_LcpActiveSetTransition/Standard/(Direct|MPRGP|Baraff)|BM_LcpBatch(Serial|Parallel)/Standard/(Direct|MPRGP|Baraff)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpCompare/Standard/MPRGP/12|BM_LcpActiveSetTransition/Standard/Baraff|BM_LcpBatchSerial/Standard/Direct/3/4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The `rg` check found no remaining `supportsProblem(solver,
getProblemSupport(...))` registration patterns in `bm_lcp_compare.cpp`.
- The benchmark-list check rebuilt `BM_LCP_COMPARE` and preserved concrete
  standard rows for Direct, MPRGP, and Baraff across single, active-set, and
  serial/parallel batch families; Direct remained absent from the active-set
  transition row.
- The short benchmark execution reported `contract_ok=1` for
  `BM_LcpCompare/Standard/MPRGP/12`,
  `BM_LcpActiveSetTransition/Standard/Baraff`, and
  `BM_LcpBatchSerial/Standard/Direct/3/4`.
- `pixi run lint`: passed.

## Grouped Batch Benchmark Support-Routing Checkpoint

The latest implementation checkpoint aligns grouped batch benchmark
registration with concrete solver support:

- `BM_LcpGroupedBatchSerial` and `BM_LcpGroupedBatchParallel` now build the
  exact generated grouped batches for the published two- and three-variant rows
  and register only rows whose concrete problem list is accepted by the solver's
  `supportsProblem(problem)` predicate.
- The benchmark remains scoped to Jacobi and PGS to preserve the existing
  CPU/CUDA-comparable surface, but the published row set is no longer gated by
  only the manifest-level problem family.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpGroupedBatch(Serial|Parallel)/(Standard|Boxed|FrictionIndex)/(Jacobi|Pgs)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpGroupedBatchSerial/Standard/Jacobi/2|BM_LcpGroupedBatchParallel/FrictionIndex/Pgs/2' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
pixi run lint
```

Observed results:

- The benchmark-list check rebuilt `BM_LCP_COMPARE` and listed 24 grouped
  serial/parallel rows: Jacobi and PGS for standard, boxed, and friction-index
  families, with two- and three-variant grouped rows for each.
- The short benchmark execution reported `contract_ok=1` for
  `BM_LcpGroupedBatchSerial/Standard/Jacobi/2` and
  `BM_LcpGroupedBatchParallel/FrictionIndex/Pgs/2`.
- `pixi run lint`: passed.

## 2026-06-11 Final No-Verification Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only,
with no further verification. This section captures the branch state for a
fresh Claude/Codex session.

Branch state before this docs-only hand-off checkpoint:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local implementation HEAD:
  `fe5b70b32cc1 Route generated LCP coverage by concrete support`.
- Previous implementation checkpoint:
  `f86e353df2f Gate heavyweight LCP contact benchmarks concretely`.
- Remote-tracking branch in this checkout:
  `origin/feature/lcp-solver-interface-demos` at
  `737b9c95c11 Document final LCP handoff checkpoint`.
- Local status before this docs-only edit:
  `feature/lcp-solver-interface-demos` was ahead of the remote-tracking branch
  by two commits.
- `main` was fetched over HTTPS from `https://github.com/dartsim/dart.git` to
  `7d05d7b9ea72`; `git merge --no-edit FETCH_HEAD` reported
  `Already up to date.`
- An SSH fetch from `origin` failed in this environment because GitHub port 22
  was unreachable; HTTPS fetch succeeded.
- No PR was associated with the branch when checked earlier in the session.

Current first-parent implementation stack before this docs-only checkpoint:

```text
fe5b70b32cc Route generated LCP coverage by concrete support
f86e353df2f Gate heavyweight LCP contact benchmarks concretely
737b9c95c11 Document final LCP handoff checkpoint
8f0242c2442 Filter LCP contact benchmark rows concretely
4c63db30bd7 Filter LCP benchmark args concretely
be4643d1743 Document consolidated LCP handoff state
```

Important hand-off constraints:

- The broad LCP objective is still open. Do not mark this dev task complete
  only because the branch is consolidated and published.
- Treat verification notes below as historical evidence for their named
  implementation checkpoints. They are not evidence for this docs-only
  hand-off checkpoint.
- A fresh session should fetch
  `feature/lcp-solver-interface-demos`, read this file and
  `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`, then continue from one
  bounded remaining LCP support-routing, solver-domain, benchmark, or py-demo
  gap.

Likely next bounded continuations:

- Audit `tests/unit/math/lcp/test_all_solvers_smoke.cpp` before changing it:
  some manifest-level checks there may intentionally test solver manifest
  categories rather than concrete native routing.
- Continue the solver documentation and py-demo coverage audit only after
  identifying a concrete mismatch between documented native domains,
  `supportsProblem(problem)`, benchmark rows, and demo metadata.

## Generated Coverage Support-Routing Checkpoint

The latest implementation checkpoint aligns generated LCP correctness coverage
with the concrete solver support predicate used by demos and benchmarks:

- `tests/unit/math/lcp/test_lcp_generated_coverage.cpp` now creates the solver
  and calls `supportsProblem(testCase.problem)` before running each generated
  case.
- The older manifest-family gate and Direct-only size special case were removed
  from the shared `solverShouldRun(...)` path. Direct size support, Baraff
  PSD-only native support, and MPRGP SPD-only native support are now all routed
  through the same solver predicates as public callers.

Verification for this checkpoint:

```bash
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_generated_coverage --parallel "$JOBS"
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_math_lcp_math_lcp_lcp_generated_coverage$'
pixi run lint
```

Observed results:

- `UNIT_math_lcp_math_lcp_lcp_generated_coverage` rebuilt successfully.
- CTest `^UNIT_math_lcp_math_lcp_lcp_generated_coverage$`: passed.
- `pixi run lint`: passed.

## Heavyweight Contact Benchmark Support-Gating Checkpoint

The latest implementation checkpoint replaces the remaining manifest-only
contact benchmark gates without making benchmark registration construct the
largest dense contact fixtures:

- `BM_LcpWorldBoxContact/FrictionIndex` now builds a one-box concrete support
  probe and registers dense box-count rows only for dense-box-scoped solvers
  whose `supportsProblem(problem)` accepts that concrete contact packet.
- `BM_LcpArticulatedUnifiedContact/FrictionIndex` now builds one-contact
  support probes for the ground, rigid-impact, and cross-link articulated cases
  and registers each case only for solvers that accept the concrete probe.
- `BM_LcpWorldContactBatchSerial/FrictionIndex` and
  `BM_LcpWorldContactBatchParallel/FrictionIndex`, including the stress-stack
  and contact-pipeline-32 families, now check each generated batch's concrete
  problem list through `supportsProblem(problem)` before publishing serial or
  parallel rows.
- `BM_LcpWorldBoxContactBatchSerial/FrictionIndex` and
  `BM_LcpWorldBoxContactBatchParallel/FrictionIndex` now build a one-box
  batch support probe and publish dense box batch rows only for dense-box-scoped
  solvers that accept that concrete batch.

Scope note: the dense box-contact and articulated unified-contact families
still avoid exact per-argument generation of the 256-contact fixtures during
benchmark listing. The registration gate is concrete and representative for
the contact family; the benchmark body remains the source of truth for each
large fixture.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpWorldBoxContact/FrictionIndex/(Pgs|Admm)|BM_LcpWorldContact(Batch|StressBatch|Pipeline32Batch)(Serial|Parallel)/FrictionIndex/(Pgs|Admm)|BM_LcpWorldBoxContactBatch(Serial|Parallel)/FrictionIndex/(Pgs|Admm)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpArticulatedUnifiedContact/FrictionIndex/(Ground|RigidImpact|CrossLinkImpact)/(Pgs|Admm)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpWorldBoxContact/FrictionIndex/Pgs/1|BM_LcpArticulatedUnifiedContact/FrictionIndex/Ground/Admm/1|BM_LcpWorldContactBatchSerial/FrictionIndex/Pgs$|BM_LcpWorldBoxContactBatchSerial/FrictionIndex/Pgs/1/4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The first benchmark-list check built `BM_LCP_COMPARE` and listed PGS and
  ADMM rows for dense world-box contact, baseline/stress/contact-pipeline mixed
  world-contact batches, and dense world-box contact batches.
- The articulated benchmark-list check listed PGS and ADMM rows for ground,
  rigid-impact, and cross-link articulated unified-contact cases.
- The short benchmark execution reported `contract_ok=1` for the targeted
  dense box-contact, articulated unified-contact, mixed world-contact batch,
  and dense box-contact batch rows; the regex also matched larger dense
  box-contact and articulated rows, which also reported `contract_ok=1`.

## Critical No-Verification Hand-Off

Current branch:

- `feature/lcp-solver-interface-demos`

Local first-parent stack at the hand-off point, before this docs-only
checkpoint:

- `8f0242c2442 Filter LCP contact benchmark rows concretely`
- `4c63db30bd7 Filter LCP benchmark args concretely`
- `be4643d1743 Document consolidated LCP handoff state`
- `02c6d0acb4b Filter active-set LCP benchmark rows concretely`
- `b2e212db5c4 Add active friction-index LCP benchmark rows`
- `d143d0dc355 Document latest LCP handoff state`

Important state:

- The working tree was clean before this docs-only hand-off edit.
- Local tracking showed the branch ahead of
  `origin/feature/lcp-solver-interface-demos` by five commits because previous
  HTTPS pushes did not refresh the SSH remote-tracking ref in this checkout.
  A fresh session should inspect the remote branch directly before assuming the
  tracking ref is authoritative.
- The latest user instruction explicitly prohibited further verification. Any
  future session should treat verification data below as historical evidence
  for the checkpoint that recorded it, not as evidence for this final docs-only
  checkpoint.
- The broad LCP objective is not complete. This dev-task folder should remain
  active until remaining follow-up work is completed or moved into durable
  planning/design docs.

Resume from:

1. Fetch the branch and inspect `git status --short --branch` and
   `git log --oneline --decorate --max-count=12 --first-parent`.
2. Read this file and `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`.
3. Continue with one bounded checkpoint, preferably a remaining concrete
   support-routing gap in `tests/benchmark/lcpsolver/bm_lcp_compare.cpp`.

## World/Contact Benchmark Routing Checkpoint

The latest implementation checkpoint aligns low-cost contact-derived benchmark
registration with concrete native solver support:

- `BM_LcpWorldContact/FrictionIndex` now precomputes the 1-, 2-, and 4-contact
  separated world packets once and registers only solver/arg rows whose concrete
  packet is accepted by `supportsProblem(problem)`.
- `BM_LcpWorldStackContact/FrictionIndex` now precomputes the existing
  2- through 16-, 24-, and 32-sphere stack packets once and filters those args
  through the same concrete support predicate.
- `BM_LcpContactSolverComparisonSweep` now builds each DART 7 contact-pipeline
  comparison packet once per case and filters the scoped comparison solvers
  (`Admm`, `Sap`, and `BoxedSemiSmoothNewton`) through concrete support.
- Dense world-box contact, articulated unified contact, and batch contact rows
  were completed by the later heavyweight contact support-gating checkpoint.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpWorldContact/FrictionIndex/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpWorldStackContact/FrictionIndex/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpContactSolverComparisonSweep/(Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpContactSolverComparisonSweep/(Admm|Sap|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpWorldContact/FrictionIndex/Dantzig/1|BM_LcpWorldStackContact/FrictionIndex/Pgs/2|BM_LcpContactSolverComparisonSweep/Admm/WorldSeparated1' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The separated world-contact and stack-contact row-list check built
  `BM_LCP_COMPARE` and preserved representative Dantzig, PGS, and
  BoxedSemiSmoothNewton rows.
- The contact-solver comparison row-list check confirmed the scoped solver set
  (`Admm`, `Sap`, `BoxedSemiSmoothNewton`) remains registered for the current
  world, stack, and articulated contact-pipeline cases.
- The short benchmark execution reported `contract_ok=1` on the targeted rows;
  the regex also matched larger `Pgs/24` and `Admm/WorldSeparated16` rows, which
  also reported `contract_ok=1`.

## Manifest And Batch Benchmark Argument Routing Checkpoint

The latest implementation checkpoint aligns manifest-generated benchmark
argument rows with concrete native solver support:

- `BM_LcpCompare` registration now precomputes candidate problem sizes and
  keeps only sizes whose generated `MakeBenchmarkProblem(...)` packet is
  accepted by the solver's concrete `supportsProblem(problem)` predicate.
- `BM_LcpBatchSerial` and `BM_LcpBatchParallel` now apply the same filter to
  every generated problem in each `MakeBenchmarkProblemBatch(...)` row.
- The existing benchmark suite shape is preserved: Direct remains limited to
  its tiny standard-LCP rows, Dantzig/Baraff/MPRGP keep their standard SPD
  rows, and boxed/friction-index rows stay available for the solvers that
  accept the generated concrete packets.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpCompare/Standard/(Direct|Dantzig|MPRGP|Baraff)|BM_LcpBatchSerial/Standard/(Direct|Dantzig|MPRGP|Baraff)|BM_LcpBatchParallel/Standard/(Direct|Dantzig|MPRGP|Baraff)'
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpCompare/(Boxed|FrictionIndex)/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpBatchSerial/(Boxed|FrictionIndex)/(Dantzig|Pgs|BoxedSemiSmoothNewton)|BM_LcpBatchParallel/(Boxed|FrictionIndex)/(Dantzig|Pgs|BoxedSemiSmoothNewton)'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpCompare/Standard/Direct|BM_LcpBatchSerial/Standard/Direct|BM_LcpCompare/FrictionIndex/BoxedSemiSmoothNewton|BM_LcpBatchParallel/Boxed/Pgs/24/4' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
```

Observed results:

- The standard row-list check built `BM_LCP_COMPARE` and listed Direct only for
  `2`/`3` single packets and `3/4` batch packets, while Dantzig, Baraff, and
  MPRGP kept their existing standard SPD rows.
- The boxed/friction-index row-list check kept representative Dantzig, PGS, and
  BoxedSemiSmoothNewton rows for single, serial batch, and parallel batch
  benchmark families.
- The short benchmark execution reported `contract_ok=1` for all representative
  affected rows that were run.

## Active-Set Transition Benchmark Routing Checkpoint

The latest implementation checkpoint aligns active-set transition benchmark
registration with concrete native solver support:

- `RegisterActiveSetTransitionBenchmarks()` now builds the concrete generated
  packet for each standard, boxed, and friction-index active-set family and
  registers only solvers whose `supportsProblem(problem)` accepts that packet.
- The previous Direct-only special case for the 16-row standard packet is gone;
  Direct is excluded by its own concrete native-support predicate.
- Baraff and MPRGP remain registered for the standard active-set transition
  packet because the generated matrix is symmetric positive-definite and
  satisfies their native predicates.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpActiveSetTransition/Standard/(Direct|Baraff|MPRGP)'
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE` and listed Baraff and MPRGP
  standard active-set transition rows, with no Direct row.
- `pixi run lint`: passed.

## Active Friction-Index Benchmark Routing Checkpoint

The latest implementation checkpoint moves the demo's active friction-index
contact benchmark metadata onto the main manifest-driven `lcp_compare` surface:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now registers
  `BM_LcpActiveFrictionIndexContact/FrictionIndex/<solver>` rows for every
  friction-index-capable manifest solver whose concrete
  `supportsProblem(problem)` accepts
  `LcpProblemFactory::activeFrictionIndexContact().problem`.
- The original narrower
  `BM_DantzigSolver_ActiveFrictionIndexContact` and
  `BM_PgsSolver_ActiveFrictionIndexContact` microbenchmark rows remain in
  `bm_lcpsolver_solvers.cpp`.
- `python/examples/demos/scenes/lcp_physics.py` now points the
  `active_friction_index_contact` representative filter at
  `BM_LcpActiveFrictionIndexContact`, so
  `representative_benchmark_command` runs rows that exist in `lcp_compare`.
- `python/tests/unit/test_py_demo_panels.py` now asserts that metadata.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpActiveFrictionIndexContact'
pixi run bm lcp_compare -- \
  --benchmark_filter='BM_LcpActiveFrictionIndexContact' \
  --benchmark_min_time=0.001s --benchmark_repetitions=1
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE` and listed 16
  friction-index-capable rows: Dantzig, Pgs, SymmetricPsor, Jacobi,
  RedBlackGaussSeidel, BlockedJacobi, BGS, NNCG, SubspaceMinimization, Apgd,
  Tgs, ShockPropagation, Staggering, Admm, Sap, and BoxedSemiSmoothNewton.
- The short benchmark execution reported `contract_ok=1` for all 16 rows.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `pixi run lint`: passed.

## 2026-06-11 Final Consolidated Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only
with no further verification. No lint, build, tests, benchmark listing, solver
execution, or implementation work was run after that instruction.

Branch state before this docs-only hand-off update:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD: `02c6d0acb4b4 Filter active-set LCP benchmark rows concretely`.
- Remote tracking branch: `origin/feature/lcp-solver-interface-demos` at
  `d143d0dc355c Document latest LCP handoff state`.
- Local branch was two commits ahead of the tracking branch:
  - `b2e212db5c4 Add active friction-index LCP benchmark rows`
  - `02c6d0acb4b Filter active-set LCP benchmark rows concretely`
- `main` was refreshed over HTTPS to `7d05d7b9ea72`, then
  `git merge --no-edit FETCH_HEAD` reported `Already up to date.`
- The working tree was clean before this docs-only hand-off update.
- No PR was associated with the branch when checked earlier in the session.

This final hand-off checkpoint should remain on the same consolidated branch,
`feature/lcp-solver-interface-demos`, together with the two latest
benchmark-routing commits. A fresh session should resume from that branch tip
and continue the remaining LCP interface/demo audit; the broad task is not
complete.

Likely next bounded continuations:

- Inspect remaining manifest-level benchmark gates in
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` and only convert them to
  concrete `supportsProblem(problem)` gates when the generated packet actually
  exercises a narrower native domain.
- Re-check solver documentation against concrete per-problem support predicates
  for any solver that still delegates internally from its public `solve()` path.
- Improve py-demo benchmark packets only when the referenced rows exist on the
  same benchmark executable surface used by
  `representative_benchmark_command`.

## Contact-Normal Benchmark Routing Checkpoint

The latest implementation checkpoint aligns contact-normal standard benchmark
registration with concrete native solver support:

- `RegisterContactNormalStandardSweepBenchmarks()` now builds each concrete
  normal-only contact packet once and registers only solvers whose
  `supportsProblem(problem)` accepts that packet.
- Current MPRGP and Baraff rows remain registered for all contact-normal sweep
  cases because the generated packets satisfy their native predicates.
- Direct's previous `contactOrShapeCount > 3` registration special case is gone;
  its rows are now limited by the same concrete support predicate and remain
  only for 1-, 2-, and 3-row normal problems.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_LcpContactNormalStandardSweep/(MPRGP|Baraff|Direct)'
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE`, listed all current Baraff
  and MPRGP contact-normal standard rows, and listed Direct only for the
  concrete 1-, 2-, and 3-row normal packets.
- `pixi run lint`: passed.

## Singular-Degenerate Benchmark Routing Checkpoint

The previous implementation checkpoint aligns singular-degenerate benchmark
registration with concrete native solver support:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` now has a benchmark-local
  `SolverSupportsConcreteProblem(...)` helper that instantiates the manifest
  solver and asks `supportsProblem(problem)` on the generated packet.
- `SolverShouldRunSingularDegenerateBenchmark(...)` still applies the scoped
  solver allowlists, but now intersects them with concrete support for the
  actual singular-degenerate packet.
- The singular-degenerate standard packets are symmetric positive-semidefinite
  but rank-deficient. Baraff remains registered as native; MPRGP no longer gets
  native singular-degenerate standard rows that would run through its Dantzig
  fallback.

Verification for this checkpoint:

```bash
pixi run bm lcp_compare -- --benchmark_list_tests=true \
  --benchmark_filter='BM_Lcp.*SingularDegenerate.*/(MPRGP|Baraff)'
pixi run lint
```

Observed results:

- The benchmark-list check built `BM_LCP_COMPARE` and listed Baraff
  singular-degenerate standard and standard-batch rows, with no MPRGP rows.
- `pixi run lint`: passed.

## Baraff Support Predicate Checkpoint

The latest implementation checkpoint makes Baraff's per-problem support
predicate match its documented symmetric-PSD native path:

- `dart/math/lcp/pivoting/baraff_solver.hpp` and `.cpp` now override
  `supportsProblem(problem, standardTolerance)` while preserving the base
  overloads.
- Baraff now reports native support only for standard problems with symmetric
  positive-semidefinite matrices.
- Boxed, friction-indexed, non-symmetric, and indefinite packets now delegate to
  Dantzig through the unified `solve()` path before entering the native Baraff
  active-set loop.
- C++ and dartpy tests cover the SPD native case, PSD native case, boxed false
  case, non-symmetric false case, indefinite false case, and fallback solve
  success for an indefinite standard packet.

Verification for this checkpoint:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=$N cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy --parallel "$N"
CTEST_PARALLEL_LEVEL=1 ctest --test-dir build/default/cpp/Release \
  --output-on-failure -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$' -j 1
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
```

Observed results:

- Targeted C++ build: passed.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke`: passed.
- `python/tests/unit/math/test_lcp.py`: 61 tests passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `scripts/check_lcp_solver_roster.py`: passed.

## MPRGP Support Predicate Checkpoint

The MPRGP support predicate checkpoint makes MPRGP's per-problem support
predicate match its actual native path:

- `dart/math/lcp/other/mprgp_solver.hpp` and `.cpp` now override
  `supportsProblem(problem, standardTolerance)` while preserving the base
  overloads.
- MPRGP now reports native support only for standard problems with symmetric
  matrices and, by default, a successful positive-definite factorization check.
- Boxed, friction-indexed, non-symmetric, and non-positive-definite packets can
  still be solved through fallback delegation, but no longer appear as native
  MPRGP rows in Python/demo capability reporting.
- C++ and dartpy tests now cover the SPD native case, boxed false case,
  non-symmetric false case, indefinite false default case, and the C++ member
  parameter case where `checkPositiveDefinite = false`.

Verification for this checkpoint:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=$N cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_all_solvers_smoke dartpy --parallel "$N"
CTEST_PARALLEL_LEVEL=1 ctest --test-dir build/default/cpp/Release \
  --output-on-failure -R '^UNIT_math_lcp_math_lcp_all_solvers_smoke$' -j 1
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run python scripts/check_lcp_solver_roster.py
```

Observed results:

- Targeted C++ build: passed.
- `UNIT_math_lcp_math_lcp_all_solvers_smoke`: passed.
- `python/tests/unit/math/test_lcp.py`: 60 tests passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `scripts/check_lcp_solver_roster.py`: passed.

## Source-Layout Docs Checkpoint

The source-layout docs checkpoint, `7f4b0227eaf Align LCP docs with snake case
headers`, realigns the LCP background docs with the actual DART 7 source
layout:

- `docs/background/lcp/02_overview.md` now lists snake_case `dart/math/lcp`
  solver header/source paths in the implementation table, repository layout,
  section headings, and completion checklist.
- `docs/background/lcp/03_pivoting-methods.md`,
  `docs/background/lcp/05_newton-methods.md`, and
  `docs/background/lcp/06_other-methods.md` now use snake_case include paths in
  C++ snippets.
- `scripts/check_lcp_solver_roster.py` now scans LCP background docs for
  documented LCP header/source paths and fails if those paths do not exist.

Verification for this checkpoint:

```bash
pixi run python scripts/check_lcp_solver_roster.py
pixi run lint
```

Observed results:

- `pixi run python scripts/check_lcp_solver_roster.py`: passed.
- `pixi run lint`: passed.

## Critical Hand-Off Snapshot

The user explicitly stopped implementation work and requested hand-off only
with no further verification. No lint, build, tests, benchmark listing, or
solver execution was run after that final instruction.

Branch state at the start of this hand-off update:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD before this docs-only hand-off update:
  `b553a8ca444 Report Baraff native support precisely`.
- Remote tracking branch before this docs-only hand-off update:
  `origin/feature/lcp-solver-interface-demos` at
  `f3436654bbd Document critical LCP handoff state`.
- Local branch was two commits ahead of the tracking branch before this
  docs-only hand-off update.
- `origin/main` was refreshed over HTTPS and `git merge --no-edit origin/main`
  reported `Already up to date.`
- This docs-only hand-off update is intended to be committed and pushed to the
  same consolidated branch after capture.

Benchmark-routing audit notes:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` still has registrations that
  gate rows with manifest-level family support such as
  `dart::test::supportsProblem(solverEntry, LcpProblemSupport::Standard)`.
- Manifest-level gates are acceptable for synthetic packet families whose
  generated packets are already known to match the scoped solvers' native
  domains, but concrete generated packets should use
  `supportsProblem(problem)` before being exposed as native benchmark rows.
- The singular-degenerate standard registration gap was addressed by filtering
  rows through `supportsProblem(problem)` for the concrete generated packet.
- The contact-normal standard registration gap was addressed by filtering rows
  through `supportsProblem(problem)` for each concrete generated normal-only
  contact packet.
- Do not refactor every manifest support gate mechanically. Generic standard
  rows produced by `MakeBenchmarkProblem(Standard, size)`,
  `MakeStandardSpdProblem`, MPRGP SPD-check sweeps, interior-point path sweeps,
  and mild/near-singular SPD builders may already be correct.

## Representative Benchmark Command Checkpoint

The latest implementation checkpoint exposes a practical benchmark-suite
command in the LCP py-demo metadata:

- `benchmark_command` remains the quick smoke command for
  `BM_LCP_COMPARE_SMOKE`.
- `representative_benchmark_filter` is derived from every
  `_BENCHMARK_PACKET_ROWS[*]["benchmark_filter"]` token in table order.
- `representative_benchmark_command` wraps that filter as a runnable
  `pixi run bm lcp_compare` command.
- `python/tests/unit/test_py_demo_panels.py` now asserts the representative
  filter exactly matches the union of the benchmark packet table filters.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `pixi run lint`: passed.

## Billiard Invariant Plot Checkpoint

The latest implementation checkpoint adds GUI plots for billiard momentum,
kinetic-energy, and symmetry invariant histories in the LCP py-demo:

- `python/examples/demos/scenes/lcp_physics.py` now emits live plot streams for
  sequential impulse and boxed-LCP billiards:
  - `Sequential billiard momentum error`
  - `Boxed LCP billiard momentum error`
  - `Sequential billiard energy error`
  - `Boxed LCP billiard energy error`
  - `Sequential billiard symmetry error`
  - `Boxed LCP billiard symmetry error`
- `python/tests/unit/test_py_demo_panels.py` asserts those plot events are
  exposed by the demo panel.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run lint
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- `pixi run lint`: passed.

## Direct Support Checkpoint

This checkpoint tightened Direct solver native support reporting:

- `LcpSolver::supportsProblem(problem)` becomes virtual.
- `DirectSolver::supportsProblem(problem, standardTolerance)` reports native
  support only for standard packets with `problem.size() <= 3`.
- `DirectSolver::solve()` remains unchanged and can still delegate larger
  standard packets to Dantzig.
- The LCP py-demo representative-suite metadata counts Direct as delegated for
  the 4-row near-singular standard case and the 12-row moderate-scale standard
  case.

Files changed in this checkpoint:

- `CHANGELOG.md`
- `dart/math/lcp/lcp_solver.hpp`
- `dart/math/lcp/pivoting/direct_solver.cpp`
- `dart/math/lcp/pivoting/direct_solver.hpp`
- `docs/background/lcp/02_overview.md`
- `docs/dev_tasks/lcp_solver_interface_demos/README.md`
- `docs/dev_tasks/lcp_solver_interface_demos/RESUME.md`
- `docs/onboarding/python-bindings.md`
- `python/tests/unit/math/test_lcp.py`
- `python/tests/unit/test_py_demo_panels.py`
- `tests/unit/math/lcp/test_all_solvers_smoke.cpp`

Verification for the Direct support checkpoint:

```bash
pixi run lint
cmake --build build/default/cpp/Release \
  --target UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  --clean-first --parallel "$JOBS"
ninja -C build/default/cpp/Release -j "$JOBS" \
  UNIT_math_lcp_math_lcp_additional_solvers \
  UNIT_math_lcp_math_lcp_all_solvers_smoke \
  UNIT_math_lcp_math_lcp_dantzig_misc \
  UNIT_math_lcp_math_lcp_dantzig_solver \
  UNIT_math_lcp_math_lcp_dantzig_vs_ode \
  UNIT_math_lcp_math_lcp_lcp_comparison_harness \
  UNIT_math_lcp_math_lcp_lcp_edge_cases \
  UNIT_math_lcp_math_lcp_lcp_generated_coverage \
  UNIT_math_lcp_math_lcp_lcp_newton_solvers \
  UNIT_math_lcp_math_lcp_lcp_projection_solvers \
  UNIT_math_lcp_math_lcp_lcp_solvers_stress \
  UNIT_math_lcp_math_lcp_lcp_problems \
  UNIT_math_lcp_math_lcp_lcp_types \
  UNIT_math_lcp_math_lcp_lcp_validation_and_solvers \
  UNIT_math_lcp_math_lcp_lemke \
  UNIT_math_lcp_math_lcp_pgs \
  UNIT_math_lcp_math_lcp_pivot_matrix
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^UNIT_math_lcp_' -j "$JOBS"
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed results:

- `pixi run lint`: passed.
- `UNIT_math_lcp_math_lcp_lcp_generated_coverage`: passed after a clean rebuild
  removed stale vtable state from the local incremental build.
- `ctest -R '^UNIT_math_lcp_'`: 17/17 tests passed.
- `python/tests/unit/math/test_lcp.py`: 59 tests passed.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

## Previous Hand-Off Snapshot

The current session was stopped for hand-off only after the billiard invariant
plot checkpoint. No implementation work and no verification were run after the
user requested: "stop and focus on hand-off for all the current work without any
further verification."

Branch state at hand-off start:

- Current branch: `feature/lcp-solver-interface-demos`.
- Local HEAD before this docs-only hand-off update:
  `cae4efcce30 Plot LCP billiard invariants`.
- Remote tracking branch before the hand-off docs update:
  `origin/feature/lcp-solver-interface-demos` at
  `b2f5632b277 Expose LCP problem validation diagnostics`.
- Local branch was six commits ahead of the tracking branch before this
  docs-only hand-off update.
- The earlier SSH fetch/push path failed on `github.com:22`, but a later HTTPS
  fetch of `origin/main` succeeded, and `origin/main` was confirmed to be an
  ancestor of `HEAD` before the contact-pipeline metadata checkpoint.

Interrupted next-slice reconnaissance, with no code changes made:

- The next bounded gap had shifted from solver coverage metadata to a practical
  representative benchmark command.
- `python/examples/demos/scenes/lcp_physics.py` still has a smoke-only command:
  `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`.
- The benchmark packet table already contains the representative per-packet
  filters, including active-set transitions, active friction-index contact,
  contact-pipeline sweeps, normal-only contact sweeps, degenerate and
  near-singular packets, batch scaling, billiards, stack, card-pile, and
  articulated-contact rows.
- A likely next bounded patch is to add separate metadata such as
  `representative_benchmark_filter` and `representative_benchmark_command`
  built from `_BENCHMARK_PACKET_ROWS`, while keeping `benchmark_command` as the
  quick smoke command for compatibility.
- Suggested test shape in `python/tests/unit/test_py_demo_panels.py`: split the
  representative filter on `|` and compare it with the union of every
  benchmark packet row filter token. Keep the smoke assertion for
  `BM_LCP_COMPARE_SMOKE`.

## Contact-Pipeline Metadata Checkpoint

The continuation after the hand-off snapshot kept the exact
`active_friction_index_contact` row pointed at its two-solver regression
benchmark, then added separate py-demo benchmark packet rows for the broader
DART 7 contact-pipeline comparisons registered in `bm_lcp_compare.cpp`:

- `contact_solver_comparison_sweep`:
  `BM_LcpContactSolverComparisonSweep|BM_LcpStaggeringContactPipelineSweep`
  for Staggering, ADMM, SAP, and BoxedSemiSmoothNewton contact-pipeline
  fixtures.
- `contact_normal_standard_sweep`: `BM_LcpContactNormalStandardSweep` for
  normal-only standard contact subproblems across standard-capable solvers.

Verification for this checkpoint:

```bash
git fetch https://github.com/dartsim/dart.git main:refs/remotes/origin/main
git merge-base --is-ancestor origin/main HEAD
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests \
  --benchmark_filter='BM_LcpContactSolverComparisonSweep|BM_LcpStaggeringContactPipelineSweep|BM_LcpContactNormalStandardSweep'
pixi run lint
```

Observed results:

- `origin/main` fetched successfully over HTTPS and is an ancestor of `HEAD`.
- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- Benchmark listing built `BM_LCP_COMPARE` and listed the referenced sweep
  benchmarks.
- `pixi run lint`: passed.

## Billiard Symmetry Metric Checkpoint

The LCP py-demo already tracked billiard momentum and kinetic-energy error.
This checkpoint adds `billiard_symmetry_error`, measured as the maximum lateral
drift of either billiard ball from its initial collision line, so the live
headless and GUI surfaces directly expose the symmetry invariant called out in
the LCP representative-example goal.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed result:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

## Stack Benchmark Filter Checkpoint

The high-mass-ratio stack live packet previously pointed only at
`BM_LcpWorldStackStep_BoxedLcp`. This checkpoint updates the demo benchmark row
to include `BM_LcpWorldStackContact/`, which is the manifest-driven
friction-index stack contact benchmark registered for the solver roster, while
keeping the boxed world-step benchmark for the live rollout analog.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
pixi run bm lcp_compare -- --benchmark_list_tests \
  --benchmark_filter='BM_LcpWorldStackContact/|BM_LcpWorldStackStep_BoxedLcp'
```

Observed results:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.
- Benchmark listing showed `BM_LcpWorldStackContact/FrictionIndex/<solver>`
  rows and `BM_LcpWorldStackStep_BoxedLcp` rows.

## Billiard Invariant Plot Checkpoint

The live LCP panel now plots billiard momentum error, kinetic-energy error, and
symmetry error histories for both the sequential impulse and boxed-LCP worlds.
This keeps the table values and GUI time histories aligned for the billiards
dealbreaker example.

Verification for this checkpoint:

```bash
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/test_py_demo_panels.py -q
```

Observed result:

- `python/tests/unit/test_py_demo_panels.py`: 43 tests passed.

## Verification Snapshot

Latest friction-coefficient validation checkpoint was verified before the
current stop-only hand-off with:

```bash
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS \
  CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-lcpsolver
PYTHONPATH=build/default/cpp/Release/python:python \
  pixi run python -m pytest python/tests/unit/math/test_lcp.py -q
pixi run lint
```

Observed results:

- `pixi run test-lcpsolver`: 17/17 tests passed.
- `python/tests/unit/math/test_lcp.py`: 62 tests passed.
- `pixi run lint`: passed.

No verification was run after the user's final stop-only instruction.

Earlier branch checkpoints also ran the LCP demo panel tests and active
friction benchmark smoke rows; re-run those if the next change touches demo
metadata, packet generation, or benchmark rows.

## Immediate Next Steps

1. Resume on `feature/lcp-solver-interface-demos` and inspect
   `git status --short --branch` plus
   `git log --oneline --decorate --max-count=15`.
2. Continue the broader LCP interface/demo audit from the next concrete gap.
3. If publishing later, fetch/merge latest `origin/main` before any push and do
   not push without explicit approval.
