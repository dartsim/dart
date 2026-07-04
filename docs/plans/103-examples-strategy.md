# PLAN-103: Examples Strategy (Python-First)

- Operating state: `PLAN-103` in [`dashboard.md`](dashboard.md)
- Outcome: Python is DART's primary, growing DART 7 example surface through the
  interactive `py-demos` workspace, headless/capture modes, and a notebook
  gallery. C++ `dart-demos` is the smaller World-only companion. Legacy DART 6
  demo implementations and cross-language golden parity fixtures have been
  removed from the demo surfaces; current follow-ups grow World-native examples,
  planned-port placeholders, and visual evidence packets.
- Related: PLAN-102 (C++ `dart-demos`, Complete — frozen by this plan),
  PLAN-012 (cloud dartpy tutorials — consumes this plan's scene modules),
  PLAN-101 (`dartsim` editor — a retire-later precondition),
  [`103-examples-strategy/rigid-body-visual-verification.md`](103-examples-strategy/rigid-body-visual-verification.md).
- Architecture owner (cross-language examples policy): this file while active;
  the C++ app architecture stays in
  [`../design/demos_app.md`](../design/demos_app.md).

This file owns scope, the resolved decisions, workstreams, acceptance criteria,
and open gaps. `dashboard.md` owns priority, status, horizon, dimension, next
step, and gate.

## Why

DART's north star puts "easy start" first, and the Python research toolkit
(`dartpy`) is the surface most researchers actually use (RL, robotics, ML). The
initial pivot made Python the growing example surface and froze C++ growth. The
DART 7 pruning pass then removed legacy DART 6 demo implementations and
fixture-only cross-language parity from the demo surfaces, keeping the examples
focused on World-native scenes, planned ports, and visual evidence users can
run locally.

## Decision (locked — not re-litigated here)

- Python is the **primary / growing** example surface; C++ is the smaller
  World-only companion.
- Current common examples are hand-maintained World-native rows, not generated
  mirrors. The **rendering shell is language-specific and not ported**.
- Modernize **content** Python-first through World-native scenes, planned-port
  placeholders, and evidence sidecars before promoting anything to the smaller
  C++ companion.
- Python's consolidated "demos" surface is a scene-registry workspace plus a
  notebook gallery. `pixi run py-demos` opens the Filament + ImGui multi-scene
  viewer by default, while `--headless`, `--cycle-scenes`, `--screenshot`, and
  `py-demo-capture` keep CI smoke and visual evidence paths noninteractive.
  `dartpy.gui.run_demos` is the constrained examples host; **no Python-side
  scene authoring API is added**.
- C++ `dart-demos` is kept smaller and retired **later** only when both the
  Python surface covers the breadth and the `dartsim` editor (PLAN-101) can open
  curated example scenes interactively. Renderer regression coverage is
  independent (`dart/gui/detail/scenes`), so retiring `dart-demos` later loses no
  renderer coverage.
- Drift prevention is now through registry cycle smokes, focused scene tests,
  visual capture evidence, and sidecar documentation. The older golden parity
  fixtures were removed during the DART 7 catalog pruning.

## Scope

In scope: the Python scene-registry runner/workspace and scene modules; the
headless smoke/capture modes; the notebook gallery; modernized Python-first
World content; planned-port placeholders; visual evidence sidecars; the
doc/source-of-truth updates; the C++ companion + retire-later checklist.

Out of scope: a general-purpose Python scene authoring/viewer binding; deleting
C++ examples now; growing the C++ `dart-demos` scene set; changes to
`dart/gui/detail/scenes` renderer fixtures (owned by the renderer tests); the
Colab publication mechanics owned by PLAN-012 (this plan supplies the scene
modules notebooks import).

## Resolved Decisions

1. **Location + runner CLI.** Scene modules and the runner live in
   `python/examples/demos/` as an importable package (`scenes/` + an ordered
   registry + a `__main__` CLI). The notebook gallery lives in
   `python/tutorials/` and **imports** the scene modules from
   `python/examples/demos` so scene logic is single-sourced. The runner mirrors
   the C++ demo shape where useful: `python -m examples.demos --scene <id>` and
   `--cycle-scenes`, plus `--frames N`, `--screenshot <path>`, `--headless`, and
   `--list`. `pixi run py-demos` opens the interactive workspace unless
   headless flags request a noninteractive smoke/capture run.
2. **Current catalog shape.** `py-demos` is the DART 7 World demo surface:
   World rigid body, AVBD rigid constraints, planned World ports, robot/control
   placeholders, Rigid IPC, PLAN-083 placeholders, simulation replay,
   variational integrators, differentiable scenes, VBD, and IPC deformable.
   Legacy DART 6 demo implementations are not kept alive in the catalog; high
   value ports are represented by lightweight planned rows until they become
   World-native scenes.
3. **Drift prevention + source of truth.** The ordered registry, focused Python
   tests, `py-demo-capture` evidence, and plan sidecars are the source of truth.
   Common examples are manually maintained. Python growth is not blocked on a
   C++ mirror, and the smaller C++ companion is not used as the primary drift
   oracle.
4. **Golden parity retirement.** The earlier cross-language JSON golden fixtures
   and parity smokes were removed during DART 7 catalog pruning. Do not add new
   golden parity fixtures as routine example work; add focused assertions or
   visual evidence to the scene that owns the behavior instead.
5. **Phasing.**
   - **Foundation:** Python runner + registry, `pixi run py-demos`, headless
     cycle smoke, and notebook import shape landed.
   - **DART 7 pruning:** legacy DART 6 demo implementations and golden parity
     fixtures were removed from the demo surfaces; planned World-port rows keep
     visible gaps discoverable without carrying stale implementations.
   - **Visual evidence packets:** current content growth is recorded through
     durable sidecars such as the rigid-body visual-verification packet, with
     dev-task folders used only while a multi-session packet is active.
   - **Retire C++ `dart-demos`:** gated by the retire-later checklist; not now.
6. **Doc/source-of-truth updates.** This numbered plan + the dashboard PLAN-103
   entry own policy and status. PLAN-012 consumes these scene modules for
   notebooks; PLAN-101 remains a retire precondition; `docs/design/demos_app.md`
   owns the C++ app architecture. Detailed evidence packets live under
   `docs/plans/103-examples-strategy/`.
7. **Retire-later checklist for C++ `dart-demos`** (ALL must hold; see below).

## Workstreams

See Decision 5 for the current phase shape. The foundational runner, notebook
import path, and DART 7 pruning are landed; the
`docs/dev_tasks/examples_strategy/` and
`docs/dev_tasks/py_demos_framework/` tracking folders were retired into this
plan and its rigid-body sidecar. Current multi-session evidence packets may use
their own temporary dev-task folders, but durable results should land under
`docs/plans/103-examples-strategy/`.

## Acceptance Criteria

- `python -m examples.demos` runs the interactive workspace by default, supports
  headless `--scene`/`--cycle-scenes`/`--frames`/`--screenshot`, lists scenes
  from an ordered registry, and `pixi run py-demos` works with pytest-gated
  cycle, full-catalog smoke, panel, and render guards.
- Focused scene tests and capture smokes cover behavior that would otherwise
  drift silently; new high-value rows have a clear test/capture owner.
- The DART 7 World catalog exists as Python scenes or planned rows; the notebook
  gallery imports the scene modules (no scene-logic duplication) and runs under
  the PLAN-012 Colab smoke.
- C++ `dart-demos` is unchanged (frozen) and still builds/cycles; no C++ scenes
  were added; the retire-later checklist is recorded and unmet items are explicit.
- Docs updated per Decision 6; `pixi run lint`, `check-docs-policy`, and the
  Python gates (`pixi run test-py`) pass for touched surfaces.

## Gate

See `dashboard.md` PLAN-103. Objective-specific proof: the Python
runner/workspace list path, full-catalog `py-demos-smoke`,
`py-demos-render-smoke`, headless cycle, and capture smokes are green; focused
tests own scene-specific behavior; the notebook gallery imports (not copies)
the scene modules; C++ stays smaller with the retire checklist tracked.

## Retire-Later Checklist (C++ `dart-demos`)

Retire `examples/demos` (the C++ app), `pixi run demos`, and the
`run_cpp_example.py` `demos` spec only when ALL hold. Current status (Phase 5
explicit "not now"):

1. **NOT MET.** Python demos cover the DART 7 World surface broadly, but the
   retire decision still needs the high-value planned rows to become usable
   World-native demos or be explicitly retired. The Python registry spans World
   rigid body, AVBD source-demo and rigid/articulated constraint rows, planned
   World ports, robot/control placeholders, Rigid IPC, PLAN-083 placeholders,
   replay, variational, differentiable, VBD, and IPC deformable categories.
   Planned rows keep remaining IK, puppet, SIMBICON, operational-space-control,
   and mobile-manipulation gaps visible without preserving legacy DART 6
   implementations in the catalog. The old
   collision-sandbox placeholder is retired to the concrete
   `rigid_contact_inspector`, `rigid_collision_query_options`, and
   `rigid_collision_casts` GUI rows.
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
5. **REMOVED AS A GATE.** Cross-language golden parity fixtures are no longer a
   retire precondition after the DART 7 pruning pass. Keep drift checks close to
   the scene that owns the behavior: focused Python tests, C++ companion smokes
   where the row exists in C++, and visual capture packets for user-facing GUI
   workflows.

Until conditions 1–3 are met, C++ `dart-demos` stays frozen-but-present.

## Open Gaps

- Colab single-source install: how notebooks in `python/tutorials/` import
  `python/examples/demos` scene modules in a managed runtime (packaged import vs
  `%pip install` from the repo subdirectory) is shared with PLAN-012.
- Planned World-port rows need per-row promotion or explicit retirement before
  the C++ companion can be retired.

## Landed State

The Python runner, interactive workspace, notebook import shape, DART 7 pruning,
and C++ companion split are landed. Retiring C++ `dart-demos` is still "not
now" because checklist items 1–3 remain open.

- **Runner + registry.** `python/examples/demos/` is an importable package with
  an ordered registry and a `__main__` CLI (`--scene`/`--cycle-scenes`/
  `--frames`/`--screenshot`/`--list`, plus interactive/headless modes).
  `pixi run py-demos` opens the workspace; the cycle smoke is pytest-gated
  (`python/tests/integration/test_demos_cycle.py`). The current catalog spans
  World rigid body, AVBD source-demo and rigid/articulated constraint rows,
  planned World ports, robot/control placeholders, Rigid IPC, PLAN-083
  placeholders, simulation replay, variational integrators, differentiable
  scenes, VBD, and IPC deformable.
- **Catalog stability guard.** The former py-demos framework task established
  a full-catalog no-crash contract: `scripts/py_demos_smoke.py`,
  `pixi run py-demos-smoke`, and `pixi run py-demos-render-smoke` exercise the
  registered catalog's build, step, provider, panel, and real-render paths.
  Registry tests catch unregistered, duplicate, or ill-formed scene modules, so
  new scenes must join the ordered catalog intentionally.
- **DART 7 pruning.** Legacy DART 6 demo implementations and cross-language
  golden parity fixtures were removed from the demo surfaces. High-value missing
  rows appear as planned World-port placeholders until they become usable
  World-native demos. Those placeholders are still launchable GUI rows, and
  their panels name the best current route, the blocker, and the replacement
  condition so users are not left at a bare roadmap label.
- **Notebook gallery.** `python/tutorials/01_browse_demos.ipynb` imports (does
  not copy) the demo scene modules; Colab publication is PLAN-012.
- **Interactive viewer.** `pixi run py-demos` delegates to
  `dartpy.gui.run_demos`, opening the Filament + ImGui multi-scene viewer with
  the Python catalog. Python `pre_step` controllers, sx force-drag callbacks,
  and scene-specific `ScenePanel` diagnostics run inside the viewer. The default
  workspace docks `Simulation`, `Demos`, scene panels, and DART diagnostics; the
  navigator is searchable, category-grouped, and transactional on scene
  switches. `py-demo-capture --show-ui` records the same docked workspace for
  visual debugging, filters UI warm-up frames, and can emit screenshots, PNG
  sequences, and MP4s.
- **Rigid-body visual evidence.** The curated World rigid-body showcase and
  validation packet lives in
  [`103-examples-strategy/rigid-body-visual-verification.md`](103-examples-strategy/rigid-body-visual-verification.md).
  It covers baseline dynamics, external and point-load semantics, material
  response, contact inspection, collision query filtering, collision casts,
  solver and executor comparisons, contact-solver policy, friction thresholds,
  stack stability, contact-rich manipulation, prescribed-motion kinematic
  drivers, joint constraints, motor/limit behavior, passive joint diagnostics,
  screw-joint pitch coupling, and loop closures. This remains an examples
  workspace, not a Python-side scene authoring API.

`dartpy.gui` remains a renderer-owned submodule with descriptor/headless helpers
plus the constrained multi-scene `run_demos` examples host; no interactive
viewer _authoring_ binding is added. C++ `dart-demos` (PLAN-102) stays frozen;
design in
[`../design/demos_app.md`](../design/demos_app.md). PLAN-012 (Colab) consumes
these scene modules; PLAN-101 (editor scene loading) is a retire precondition.

## Landed Follow-Up: Rigid-Body Visual Verification

The bounded World rigid-body visual-verification workflow is now durable in
[`103-examples-strategy/rigid-body-visual-verification.md`](103-examples-strategy/rigid-body-visual-verification.md),
which owns the 36-row user-facing `py-demos` workflow, scene-panel evidence,
replay/capture coverage, focused tests, deferred API gaps, and capture-first
related packets. The root README source-checkout path points first to this
Python-first rigid-body GUI verifier, its workflow docs, and a docked capture
command before the frozen C++ companion smoke. The temporary
`docs/dev_tasks/py_demos_framework/` working folder is retired after its M0/M1
state moved into this plan, the rigid-body sidecar, the Python demo README,
tests, and changelog entries. Future rigid visual-verification follow-ups should
update the sidecar, Python demo README, registry/tests, and dashboard instead
of restoring a working dev-task folder unless a new multi-session task starts.
