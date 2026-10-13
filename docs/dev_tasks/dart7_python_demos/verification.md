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
- **API snapshots:** #3735 makes `api_surface.py --snapshot` read
  non-arithmetic enums, so the binding PRs can diff the API surface.

## 2026-10-12: WP-PD.17, Python demo launch tasks

- **Change:** `demos`, `cycle-demos` and `capture`, including Windows
  variants, build dartpy and launch `python -m examples.demos`. The task
  working directory is `python/`; the command returns to the repository root
  with both dartpy and the source catalog on `PYTHONPATH`, preserving every
  relative CLI output path, including a forwarded `--shot` override.
  `py-demos` is removed. User docs, Python scene links and Korean PO/MO
  catalogs use the Python app. The C++ app, aliases and CTests remain for
  WP-PD.18.
- **Checks passed:**
  - `pixi run config`, then `pixi run build-py-dev` on the fresh build;
  - `pixi run python -m pip install ssik` (8.1.0, environment only);
  - `pixi run lint`, `pixi run sync-ai-commands` and
    `pixi run check-ai-commands`;
  - `pixi run test-ai-infra`: 1,968 passed;
  - `pixi run cycle-demos`: 37 scenes, twice, two frames each, zero skipped;
  - `pixi run demos -- --list-scenes`: 37 unique scene IDs;
  - `pixi run capture sleeping`: repository-root `capture.png`, 640×480,
    simulation time 2.5 seconds;
  - `pixi run capture soft_bodies -- --steps 50`: the same output path,
    frame 50 and simulation time 0.05 seconds;
  - additional captures with absolute and relative paths containing spaces,
    and a forwarded `--shot` overriding the positional output;
  - `pixi run test-py`: 1,610 passed, two Python 3.15-only skips;
  - `pixi run test-gui`: all 14 CTests passed; Python GUI and integration
    tests reported 550 passed and one Python 3.15-only skip;
  - `pixi run docs-build` and `pixi run docs-build-ko`, repeated after the
    review corrections;
  - changed-file local-path scan and `git diff --check`;
  - final `pixi run lint` and `pixi run check-lint`, including the configured
    semantic AI infrastructure check.
- **Visual assessment:** `pixi run image-verdict capture.png` passed for
  Sleeping. Native image inspection showed the Sleeping scene label,
  colored stacks and native workspace. The subsequent Soft Bodies image
  showed translucent embedded shapes and frame 50. These images corroborate
  the launcher and capture output; they do not re-establish scene parity or
  dynamics correctness. The catalog cycle and Python tests provide the text
  evidence. The default 640×480 workspace clips some panel labels, but the
  scene is visible. Images remain untracked.
- **Reviews:** two role-separated read-only reviews of the final task/docs
  diff are clean after correcting the configure-task name, humanoid slider
  instructions and manual dartpy build prerequisites. Windows definitions
  were read twice; they use the multi-config dartpy path and the existing
  `build-py-dev --config` build.
- **Limitations:** Windows runtime cannot be checked on this host. No commit
  or publication is authorized. The C++ app's removal belongs to WP-PD.18.
