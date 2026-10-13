# Verification

Each entry records the date, the packet, what changed, the checks that ran,
the review passes, and any gaps. Per [the README](README.md#gates), a packet
is done only after two clean review passes on its final state.

## 2026-10-12: WP-PD.0, decision and project home

- **State:** merged in #3732.
- **Change:**
  - D31b in the DART 7 decisions, plus the program README notes on the
    milestone, release path and interim bindings.
  - A Rendering bullet in `docs/design/dart7_python_api.md`.
  - The PLAN-622 GUI-example policy, in the dashboard and in the
    deformable-body project home.
  - This project home.
- **Design evidence:**
  - read-only exploration (six slices plus a completeness critic);
  - a design workflow with four sections, an adversarial verifier per
    section, and an integration critic;
  - an independent Codex design and a Codex review of the plan.

  Every finding that changed the plan was verified against the source at
  `9e4bb4e6f`.
- **Checks:** `pixi run lint` and `pixi run check-lint` passed. The
  check-lint aggregate includes `check-ai-commands` and the semantic
  `check-ai-infra` checks. The pre-commit AI gate and the local-path gate also
  passed.
- **Reviews:** every pushed head of #3732 had two consecutive clean
  local passes (`codex review --base origin/main`, gpt-6.1-sol at ultra
  effort). The GitHub reviews of the first three heads raised findings that
  the following commits fixed: runnable parity and wheel gates, the active
  setup's retention, the architecture table, the scene batches' helper
  owners, the active context's retention, the wheel catalog, the exact
  SIMBICON files, and this record. Later heads added the drawing-callback
  non-goal, the build-tree C++ parity command, the optional inputs a
  full-catalog run needs, tinkertoy's pick-and-drag check, the Pixi
  interpreter for parity runs, a world oracle for the humanoid scenes, and
  separate capture paths for the two apps.

## 2026-10-12: WP-PD.1 to WP-PD.3, the host stack

Published as stack #3748: #3745 (WP-PD.1), #3746 (WP-PD.2) and #3747
(WP-PD.3). Each PR's Testing section holds the commands and results.

- **WP-PD.1:** the default build, `test-gui`, `test-py`, the vanilla ImGui
  lane, the Linux wheel with its headless checks, and the GUI install test
  for bundled and system ImGui all pass. Reviews raised the bundled-header
  precedence for installed consumers and a consumer test that broke system
  ImGui builds; both are fixed and documented.
- **WP-PD.2:** the four model tests, `cycle-demos`, and the GUI-off and IO-off
  lanes pass; the 16 model files move unchanged.
- **WP-PD.3:** the host tests, `cycle-demos`, `test-gui`, the memory
  diagnostics lanes, the vanilla and GUI-off lanes, the install guard and 28
  before/after command-line runs pass. A `sleeping` capture differs from its
  base only in the two FPS readouts.
- **Reviews:** two consecutive clean local passes on each final head. The
  GitHub reviews then found that a copied `examples` directory no longer
  builds `dart-demos`, now documented as source-tree only (#3746), and that
  the checker rejected `test_GuiApp` as inactive in GUI-off builds (#3747).
  Both are fixed.

## 2026-10-12: WP-PD.8 and WP-PD.9, model bindings

- **WP-PD.8 (#3737):** resting state, `HeightmapShapef` and `VoxelGridShape`
  with occupancy updates. A review found uninitialized heightmap bounds in
  C++, fixed separately in #3744. Later reviews hardened the voxel inputs
  against OctoMap's limits: points must be finite and addressable after
  float narrowing, rays must fit OctoMap's key buffer and have a nonzero
  float length, and resolution is capped at 1e6 so ray lengths cannot
  overflow float. They also fixed a C++ query that read the wrong boundary
  cell.
- **WP-PD.9 (#3751):** whole-body IK bindings plus a C++ guard for expired
  hierarchy constraints. `test`, `test-py`, the GC probes and the Gazebo gate
  pass. A review found that GC could clear an IK module that moved to a live
  Skeleton; the traversal now counts only nodes still in the hierarchy's own
  Skeleton, with a regression test.
- **API snapshots:** merged #3735 makes `api_surface.py --snapshot` read
  non-arithmetic enums, so the binding PRs can diff the API surface.
- **Release-target check:** merged #3759 skips stacked PR bases.

## 2026-10-12: WP-PD.4 (#3758), panels and scene contract

- **Tests:** `test_GuiApp` has 31 cases.
- **Teardown review:** callable destruction could add panel callbacks during
  teardown.
- **Teardown fix:** panel callbacks are cleared until none remain.
- **macOS CI failure:** caused by ImGui swapping Ctrl and Cmd under scripted
  input.
- **Input fix:** the test context turns off that swap.
- **Inputs:** the grid and heightmap fields of later batches gained ImGui's
  fast step.
- **Reviews:** two clean local passes on the final head.

## 2026-10-12: WP-PD.5 (#3761), viewer bindings

- **Tests:** `test_viewer.py` passes all 32 tests on the final head.
- **GUI:** `pixi run test-gui` passes on the final head: 14 C++ tests and
  299 Python tests passed, with 1 skipped.
- **API snapshot:** exactly 99 `dartpy.viewer` entries added.
- **Wheel:** the Linux wheel is 17.6 MB with the viewer.
- **Headless:** the wheel's headless checks pass.
- **Vanilla lane:** dartpy builds without the viewer.
- **Reviews:** two clean local passes.

## 2026-10-12: WP-PD.6 (#3755), model bindings

- **Review finding:** a dropped model left its pelvis weld in a surviving
  World.
- **Fix:** controllers handed to Python now unharness the pelvis with their
  last owner.
- **Local reviews:** two clean passes.
- **GitHub review:** clean.

## 2026-10-12: WP-PD.7, S1 and S2 (#3762, #3763, #3764)

- **World reports:** all 19 scenes match the C++ app's
  `--debug-print-world` report.
- **Seeded scenes:** the comparison uses the counts oracle.
- **Humanoids:** the comparison uses cached robot assets.
- **Captures:** every pair shows the same scene, camera and objects.
- **Reviews:** each PR had two clean local passes.

## 2026-10-12: S5 (WP-PD.14, #3765)

- **World reports:** all seven scenes match the C++ app.
- **Captures:** all seven scenes match the C++ captures.
- **Python cycle:** 29 scenes cycle.
- **C++ cycle:** all 37 scenes cycle.
- **WorldHistory:** moved into the model library with four C++ cases.
- **Reviews:** two clean local passes.
