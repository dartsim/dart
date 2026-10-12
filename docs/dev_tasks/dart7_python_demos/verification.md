# Verification

Each entry records the date, the packet, what changed, the checks that ran,
the review passes, and any gaps. Per [the README](README.md#gates), a packet
is done only after two clean review passes on its final state.

## 2026-10-12: WP-PD.0, decision and project home

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
- **Reviews:** every pushed head of #3732 first gets two consecutive clean
  local passes (`codex review --base origin/main`, gpt-6.1-sol at ultra
  effort). The GitHub reviews of the first three heads raised findings that
  the following commits fixed: runnable parity and wheel gates, the active
  setup's retention, the architecture table, the scene batches' helper
  owners, the active context's retention, the wheel catalog, the exact
  SIMBICON files, and this record.

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
- **Reviews:** two consecutive clean local passes on each final head.

## 2026-10-12: WP-PD.8 and WP-PD.9, model bindings

- **WP-PD.8 (#3737):** resting state, `HeightmapShapef` and `VoxelGridShape`
  with occupancy updates. A review found uninitialized heightmap bounds in
  C++, fixed separately in #3744.
- **WP-PD.9 (local):** whole-body IK bindings plus a C++ guard for expired
  hierarchy constraints. `test`, `test-py`, the GC probes and the Gazebo gate
  pass. A review found that GC could clear an IK module that moved to a live
  Skeleton; the traversal now counts only nodes still in the hierarchy's own
  Skeleton, with a regression test.
- **API snapshots:** #3735 makes `api_surface.py --snapshot` read
  non-arithmetic enums, so the binding PRs can diff the API surface.
