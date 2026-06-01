# PLAN-103: Examples Strategy (Python-First)

- Operating state: `PLAN-103` in [`dashboard.md`](dashboard.md)
- Outcome: Python is DART's primary, growing example surface (an interactive
  `py-demos` workspace with headless/capture modes plus a Colab notebook
  gallery); C++ examples are frozen and smaller; common scene + physics logic is
  near-identical across languages and kept honest by a thin golden-set parity
  smoke, not 1:1 duplicate maintenance.
- Related: PLAN-102 (C++ `dart-demos`, Complete — frozen by this plan),
  PLAN-012 (cloud dartpy tutorials — consumes this plan's scene modules),
  PLAN-101 (`dartsim` editor — a retire-later precondition).
- Architecture owner (cross-language examples policy): this file while active;
  the C++ app architecture stays in
  [`../design/demos_app.md`](../design/demos_app.md).

This file owns scope, the resolved decisions, workstreams, acceptance criteria,
and open gaps. `dashboard.md` owns priority, status, horizon, dimension, next
step, and gate.

## Why

DART's north star puts "easy start" first, and the Python research toolkit
(`dartpy`) is the surface most researchers actually use (RL, robotics, ML). The
C++ GUI examples were just consolidated into `dart-demos` (PLAN-102), but growing
examples in C++ does not serve the Python-first audience and doubles maintenance
for a one-maintainer project. The decided pivot makes Python the superset
example surface, modernizes example content to current robotics-research
scenarios, and freezes (does not delete) the working C++ examples.

## Decision (locked — not re-litigated here)

- Python is the **superset / primary / growing** example surface; C++ is
  **frozen and smaller** ("fewer C++ than Python" by not growing C++, not by
  deleting working code).
- Common examples are **near-identical** across languages for scene + physics
  logic (world build, joints, controllers, step loop), since `dartpy` mirrors the
  C++ API in snake_case. The **rendering shell is language-specific and not
  ported**.
- Modernize **content** Python-first (modern robotics-research scenarios), then
  mirror only a small golden subset in C++.
- Python's consolidated "demos" surface is a scene-registry workspace plus a
  notebook gallery. `pixi run py-demos` opens the Filament + ImGui multi-scene
  viewer by default, while `--headless`, `--cycle-scenes`, `--screenshot`, and
  `py-demo-capture` keep CI smoke and visual evidence paths noninteractive.
  `dartpy.gui.run_demos` is the constrained examples host; **no Python-side
  scene authoring API is added**.
- C++ `dart-demos` is **frozen now**, retired **later** only when both the Python
  surface covers the breadth and the `dartsim` editor (PLAN-101) can open curated
  example scenes interactively. Renderer regression coverage is independent
  (`dart/gui/detail/scenes`), so retiring `dart-demos` later loses no renderer
  coverage.
- Parity is a **thin golden-set smoke** (~3-5 canonical scenes), not 1:1
  duplicate maintenance.

## Scope

In scope: the Python scene-registry runner/workspace and scene modules; the
headless smoke/capture modes; the notebook gallery; the cross-language
golden-set parity smoke; modernized Python-first example content; the
doc/source-of-truth updates; the C++ freeze + retire-later checklist.

Out of scope: a general-purpose Python scene authoring/viewer binding; deleting
C++ examples now; growing the C++ `dart-demos` scene set; changes to
`dart/gui/detail/scenes` renderer fixtures (owned by the renderer tests); the
Colab publication mechanics owned by PLAN-012 (this plan supplies the scene
modules notebooks import).

## Resolved Decisions

1. **Location + runner CLI.** Scene modules and the runner live in
   `python/examples/demos/` as an importable package (`scenes/` + an ordered
   registry + a `__main__` CLI). The notebook gallery lives in
   `python/tutorials/` (the existing, currently-empty tutorials surface; aligns
   with PLAN-012) and **imports** the scene modules from `python/examples/demos`
   so scene logic is single-sourced. The runner mirrors `dart-demos`:
   `python -m examples.demos --scene <id>` and `--cycle-scenes`, plus
   `--frames N`, `--screenshot <path>`, `--headless`, and `--list`.
   `pixi run py-demos` opens the interactive workspace unless headless flags
   request a noninteractive smoke/capture run.
2. **Canonical modern set + C++ subset.** Python-first (Python-only for now):
   `legged_whole_body_control` (quadruped/humanoid balance via a whole-body
   PD/QP controller), `contact_rich_manipulation` (arm/gripper pushing or
   grasping table objects), `rl_gym_env` (a Gymnasium-style `reset`/`step`
   wrapper over an experimental World), `mpc_cartpole` (MPC-in-the-loop on a
   cart-pole/arm), `sensor_depth_segmentation` (headless depth + segmentation
   outputs via the renderer/camera math). The **golden subset** (mirrored in
   both languages) is the small set of canonical basics that already exist as
   C++ `dart-demos` scenes (see Decision 4); the modern scenarios stay
   Python-only. C++ stays frozen at its current scene set — no new C++ scenes.
3. **Drift prevention + source of truth.** Manual pairing (no code generation),
   guarded by the golden-set parity smoke. **Python is the source of truth** for
   common example content going forward (modernization is Python-first); the C++
   golden scenes are the frozen mirror. Each language's golden smoke asserts its
   scene against a shared checked-in expected-state fixture, so drift in either
   language fails that language's smoke without cross-process comparison. Only
   the golden set is maintained in lockstep; other scenes are Python-only.
4. **Golden set + parity assertions.** Golden scenes (exist in both C++ and
   Python): `hello_world` (falling box), `rigid_chain` (articulated chain
   settling), `boxes` (small contact drop), `operational_space_control`
   (PD/operational-space arm to a target). Each ships a shared
   `python/examples/demos/golden/<scene>.json` fixture of expected state
   (skeleton positions / body COM) after a fixed timestep and N deterministic
   steps. Assertions: (a) both languages match the fixture within a tight numeric
   tolerance; (b) a non-blank screenshot floor where a rendered frame exists (C++
   via `dart-demos --scene <id> --screenshot`; Python via the `dartpy.gui`
   headless descriptor → screenshot path). The fixture is generated once and
   reviewed; regenerating it is a deliberate, reviewed change.
5. **Phasing.**
   - **Phase 1 (foundation, first PR):** the Python headless runner + registry
     in `python/examples/demos/`; migrate the two existing Python examples
     (`hello_world`, `experimental_rigid_body`) into it as the first scenes;
     `pixi run py-demos`; a Python headless cycle smoke. No modernization yet.
   - **Phase 2 (parity harness):** define the golden set; add the shared
     expected-state fixtures and the C++ + Python golden smokes; extend the
     `dartpy.gui` headless surface (camera/screenshot) only as far as the
     screenshot floor needs.
   - **Phase 3 (modernize content, Python-first):** add the modern scenarios
     (Decision 2) as Python scenes.
   - **Phase 4 (notebook gallery):** notebooks in `python/tutorials/` importing
     the scene modules; Colab smoke under PLAN-012.
   - **Phase 5 (retire C++ `dart-demos`):** gated by the retire-later checklist;
     not now.
     Migration order: existing Python examples → golden parity → modern content →
     notebooks. The parity harness lands before modernization so new content has a
     drift guard. C++ is untouched except for the golden smoke.
6. **Doc/source-of-truth updates.** This numbered plan + a dashboard PLAN-103
   entry are added. The dashboard PLAN-102 entry gains a "frozen; retire under
   PLAN-103" note; PLAN-012 gains a cross-reference that its notebooks consume
   this plan's scene modules. `docs/design/demos_app.md` records the freeze +
   retire-later decision and points here. `examples/README.md` and
   `python/examples/README.md` are updated in Phase 1 to state Python is the
   primary/growing surface and point to `python/examples/demos` + the notebook
   gallery. A new PLAN-### "Examples Strategy" entry is warranted: **yes** (this
   file).
7. **Retire-later checklist for C++ `dart-demos`** (ALL must hold; see below).

## Workstreams

See Decision 5 for phase contents and ordering. Phases 1–4 are landed on `main`
(see Landed State below); the `docs/dev_tasks/examples_strategy/` tracking folder
was retired into this plan on completion. This plan is again the tracking surface
for the residual Phase-5 retire-later follow-ups.

## Acceptance Criteria

- `python -m examples.demos` runs the interactive workspace by default, supports
  headless `--scene`/`--cycle-scenes`/`--frames`/`--screenshot`, lists scenes
  from an ordered registry, and the two existing Python examples are scenes in
  it; `pixi run py-demos` works and a Python cycle smoke is CTest/pytest-gated.
- The golden set (Decision 4) has shared expected-state fixtures and passing
  C++ and Python golden smokes; modifying a golden scene's physics fails the
  smoke until the fixture is deliberately regenerated.
- The modern scenarios (Decision 2) exist as Python scenes; the notebook gallery
  imports the scene modules (no scene-logic duplication) and runs under the
  PLAN-012 Colab smoke.
- C++ `dart-demos` is unchanged (frozen) and still builds/cycles; no C++ scenes
  were added; the retire-later checklist is recorded and unmet items are explicit.
- Docs updated per Decision 6; `pixi run lint`, `check-docs-policy`, and the
  Python gates (`pixi run test-py`) pass for touched surfaces.

## Gate

See `dashboard.md` PLAN-103. Objective-specific proof: the Python
runner/workspace headless cycle and capture smokes are green; the golden-set
fixtures + dual-language smokes catch drift; the notebook gallery imports (not
copies) the scene modules; C++ stays frozen with the retire checklist tracked.

## Retire-Later Checklist (C++ `dart-demos`)

Retire `examples/demos` (the C++ app), `pixi run demos`, and the
`run_cpp_example.py` `demos` spec only when ALL hold. Current status (Phase 5
explicit "not now"):

1. **NOT MET (largely closed).** Python demos cover the breadth (>= C++
   pedagogical coverage). Today: **79 Python scenes vs 41 C++ scenes** (Python
   now exceeds C++ once the Python-first modern scenarios, the IPC-deformable
   category, and the variational/differentiable sx scenes are counted); on the
   C++-set-only axis, **39 of 41 C++ scenes** (~95 %) have a Python
   mirror. The remaining C++-only scenes fall into well-understood binding
   gaps tracked outside this gate:
   - **Viewer-callback / interactive UI** — `imgui`, `tinkertoy`,
     `simulation_event_handler`. Need Python bindings for
     `dart::gui::KeyboardAction`/Gizmo/event-handler callbacks threaded
     through `ApplicationOptions`.
   - **Custom `ConstraintBase` subclass** — `human_joint_limits`. Needs a
     trampoline binding for `dart::constraint::ConstraintBase` so a
     `NeuralJointLimitConstraint` analogue can be authored in Python.
   - **Heavyweight puppet IK / mocap controllers** — `atlas_puppet`,
     `hubo_puppet`, `fetch`, `g1_puppet`. Need gizmo + IK pendant +
     MJCF/ReadOptions package-resolver bindings.
   - **SIMBICON walking controller in C++** — `atlas_simbicon`. Controller
     and state-machine sources live under
     `tests/integration/atlas_simbicon/` and are not Python-bound.
   - **Runtime-loaded analytical IKFast** — `wam_ikfast`. Needs a
     `SharedLibraryIkFast` binding.
   - **`sx::World` deformable mesh build** — `experimental_deformable`.
     `DeformableBody`/`DeformableBodyOptions` exist; the scene also needs a
     tet-mesh construction helper (or a Python adapter to
     `dartpy.simulation_experimental.load_deformable_scene`).
2. **NOT MET.** Notebook gallery published with a green Colab smoke. Today:
   `python/tutorials/01_browse_demos.ipynb` is the seed; Colab publication +
   smoke is PLAN-012's responsibility.
3. **NOT MET.** `dartsim` editor (PLAN-101) can open curated example scenes
   interactively. Today the editor authors its own scenes; loading
   `examples/demos` content interactively is a PLAN-101 follow-up.
4. **MET.** Renderer regression coverage is independent of `dart-demos`:
   `dart/gui/detail/scenes` + `test_filament_scene_extraction` +
   `EXAMPLE_dartsim_<scene>` are owned by `dart::gui` and exercise renderer
   fixtures, not the examples (see `docs/design/demos_app.md`).
5. **MET (mechanism); at floor.** Cross-language golden parity is implemented
   end-to-end for 3 scenes (`hello_world`, `boxes`, `rigid_chain`; Python
   `test_golden_parity` + C++ `UNIT_gui_DemosGoldenParity` both pass against the
   shared fixtures within 1e-9) — at the planned "~3-5 canonical" floor.
   `operational_space_control` is mirrored in Python but deliberately excluded
   from the golden set: its controller diverges across languages (numpy vs Eigen
   operation order amplifies through the Kp gain into ~1e-3 drift over 60 steps),
   so adding it needs a deliberate tolerance/reordering decision, not a
   mechanical port.

Until conditions 1–3 are met, C++ `dart-demos` stays frozen-but-present.

## Open Gaps

- Colab single-source install: how notebooks in `python/tutorials/` import
  `python/examples/demos` scene modules in a managed runtime (packaged import vs
  `%pip install` from the repo subdirectory) is shared with PLAN-012.
- `sensor_depth_segmentation` depends on the renderer's depth/segmentation
  outputs (the fidelity-profile seam, PLAN-090) being reachable headless from
  `dartpy.gui`; confirm before adding that scene.

## Landed State (Phases 1–4, on `main`)

Phases 1–4 are merged; Phase 5 (retire C++ `dart-demos`) is the explicit "not
now" recorded in the checklist above. Verified via `pixi run py-demos --
--cycle-scenes` (exit 0), the Python + C++ golden smokes, and `pixi run lint`.

- **Runner + registry.** `python/examples/demos/` is an importable package with
  an ordered registry and a `__main__` CLI mirroring `dart-demos`
  (`--scene`/`--cycle-scenes`/`--frames`/`--screenshot`/`--list`, plus
  interactive/headless modes). `pixi run py-demos`; the cycle smoke is
  pytest-gated (`python/tests/integration/test_demos_cycle.py`). **79 Python
  scenes** across Getting Started, Visualization, Rigid Body, Collision,
  Constraints & Joints, Soft Bodies, Robots, Control & IK, Control & Modern,
  Experimental, IPC Deformable (sx), and Differentiable (sx).
- **Golden parity.** Shared JSON fixtures + dual-language smokes
  (`python/tests/unit/test_golden_parity.py`,
  `tests/unit/gui/test_demos_golden_parity.cpp`) assert 3 scenes within 1e-9.
  `examples/demos` is split into a `demos_scenes` static library + a thin
  `dart-demos` app so the tests can link the registry.
- **Modern content.** Python-first scenarios: `legged_balance`, `arm_push_box`,
  `cartpole_gym_env` (Gymnasium-style env), `cartpole_mpc` (LQR),
  `sensor_descriptors`.
- **Notebook gallery.** `python/tutorials/01_browse_demos.ipynb` imports (does
  not copy) the demo scene modules; Colab publication is PLAN-012.
- **Interactive viewer.** `pixi run py-demos` delegates to `dartpy.gui.run_demos`
  (bindings in `python/dartpy/gui/viewer.cpp`), opening the same Filament + ImGui
  multi-scene viewer as `pixi run demos` with the Python catalog; `--screenshot`
  writes a real PPM. Python `pre_step` controllers, sx force-drag callbacks, and
  scene-specific `ScenePanel` diagnostics now run inside the interactive viewer.
  The default workspace docks `Simulation`, `Demos`, scene panels, and DART
  diagnostics. The navigator is searchable, category-grouped, experimental-focus
  aware for sx/solver scenes, and transactional scene switches roll back or can
  be retargeted instead of leaving the workspace stuck on a broken candidate.
  `py-demo-capture --show-ui` records the same docked workspace for visual
  debugging, filters UI warm-up frames, and can emit screenshots, PNG sequences,
  and MP4s. This remains an examples workspace, not a Python-side scene
  authoring API.
- **Bindings added to unblock ports.** `python/dartpy/dynamics/shape.cpp`:
  `Capsule`/`Cylinder`/`Ellipsoid`/`Cone`/`Pyramid`/`LineSegment`/`Plane` shapes;
  `skeleton.cpp`: `getVelocities`/`setVelocities`/`getForces`/`getMassMatrix`/
  `getCoriolisAndGravityForces`/`setForces`; `body_node.cpp`:
  `getLinearVelocity(offset)`.
- **Deformable consolidation.** `experimental_deformable_gui` migrated into
  `dart-demos` as the `experimental_deformable` scene (Soft Bodies); the
  standalone example dir was removed and references updated (PLAN-081).

`dartpy.gui` remains a renderer-owned submodule with descriptor/headless helpers
plus the constrained multi-scene `run_demos` examples host; no interactive
viewer _authoring_ binding is added. C++ `dart-demos` (PLAN-102) stays frozen;
design in
[`../design/demos_app.md`](../design/demos_app.md). PLAN-012 (Colab) consumes
these scene modules; PLAN-101 (editor scene loading) is a retire precondition.

## Delta (in flight): `examples-strategy-breadth` branch

Phase-5 gate 1 (breadth) is being closed incrementally on top of the
landed-state baseline. The breadth-growth branch adds the following:

- **+10 Python scene mirrors** of previously C++-only `dart-demos` scenes:
  `hybrid_dynamics`, `lcp_physics`, `heightmap`, `point_cloud`, `vehicle`,
  `biped_stand`, `experimental_rigid_body_gui`, `collision_sandbox`,
  `joint_constraints`, `drag_and_drop`. Each verified under
  `pixi run py-demos -- --scene <id> --headless`. (39 of 41 C++ scenes now
  mirrored; coverage on the C++-set axis ~95 %.)
- **Shape bindings.** `python/dartpy/dynamics/shape.cpp`: `HeightmapShape`
  (double instantiation; default ctor, `set_height_field` (width/depth + flat
  list OR HeightField matrix), `get_height_field`, `set_scale`/`get_scale`,
  `get_width`/`get_depth`/`get_min_height`/`get_max_height`, `flip_y`,
  `get_static_type`) and `PointCloudShape` (visual-size ctor;
  `reserve`/`add_point`/`add_points`/`set_points`/`get_points`/
  `get_num_points`/`remove_all_points`; point-shape-type + color-mode +
  overall-color + visual-size accessors; nested `ColorMode` /
  `PointShapeType` enums).
- **External-force + COM bindings.** `body_node.cpp`:
  `addExtForce`/`setExtForce`/`clearExternalForces`. `skeleton.cpp`:
  `getConstraintForces`, `getCOM`, `clearExternalForces`. These close the
  Stable PD controller binding gap so `biped_stand` and `joint_constraints`
  port cleanly.
- **Documentation.** Retire-Later Checklist condition 1 updated to track the
  remaining C++-only scenes by their binding/infra gap (gizmo callbacks,
  custom `ConstraintBase`, puppet IK + MJCF/ReadOptions, SIMBICON state
  machine, `SharedLibraryIkFast`, tet-mesh helper for `sx::World`
  deformable). OSC golden-parity gap kept deferred; documented that the
  per-scene-tolerance path also requires updating the C++ smoke's
  hardcoded `kGoldenAbsTol` and regenerating the OSC fixture on both sides.
