# PLAN-103: Examples Strategy (Python-First)

- Operating state: `PLAN-103` in [`dashboard.md`](dashboard.md)
- Outcome: Python is DART's primary, growing example surface (a headless
  scene-registry runner plus a Colab notebook gallery); C++ examples are frozen
  and smaller; common scene + physics logic is near-identical across languages
  and kept honest by a thin golden-set parity smoke, not 1:1 duplicate
  maintenance.
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
- Python's consolidated "demos" surface is **headless**: a scene-registry runner
  plus a notebook gallery. `dartpy.gui` exposes only headless descriptors
  (renderable extraction, camera math, picking, screenshot); **no interactive
  viewer binding is added**.
- C++ `dart-demos` is **frozen now**, retired **later** only when both the Python
  surface covers the breadth and the `dartsim` editor (PLAN-101) can open curated
  example scenes interactively. Renderer regression coverage is independent
  (`dart/gui/detail/scenes`), so retiring `dart-demos` later loses no renderer
  coverage.
- Parity is a **thin golden-set smoke** (~3-5 canonical scenes), not 1:1
  duplicate maintenance.

## Scope

In scope: the Python headless scene-registry runner and scene modules; the
notebook gallery; the cross-language golden-set parity smoke; modernized
Python-first example content; the doc/source-of-truth updates; the C++
freeze + retire-later checklist.

Out of scope: any interactive `dartpy` viewer binding; deleting C++ examples now;
growing the C++ `dart-demos` scene set; changes to `dart/gui/detail/scenes`
renderer fixtures (owned by the renderer tests); the Colab publication mechanics
owned by PLAN-012 (this plan supplies the scene modules notebooks import).

## Resolved Decisions

1. **Location + runner CLI.** Scene modules and the runner live in
   `python/examples/demos/` as an importable package (`scenes/` + an ordered
   registry + a `__main__` CLI). The notebook gallery lives in
   `python/tutorials/` (the existing, currently-empty tutorials surface; aligns
   with PLAN-012) and **imports** the scene modules from `python/examples/demos`
   so scene logic is single-sourced. The runner mirrors `dart-demos`:
   `python -m examples.demos --scene <id>` and `--cycle-scenes`, plus
   `--frames N`, `--screenshot <path>`; it is always headless (`--headless`
   accepted as an explicit no-op). Add a `pixi run py-demos` task.
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

See Decision 5 for phase contents and ordering. Implementation that spans
sessions should open `docs/dev_tasks/examples_strategy/` (via `/dart-new-task`)
when Phase 1 begins; until then this plan is the tracking surface.

## Acceptance Criteria

- `python -m examples.demos` runs headless with `--scene`/`--cycle-scenes`/
  `--frames`/`--screenshot`, lists scenes from an ordered registry, and the two
  existing Python examples are scenes in it; `pixi run py-demos` works and a
  Python cycle smoke is CTest/pytest-gated.
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

See `dashboard.md` PLAN-103. Objective-specific proof: the Python headless runner

- cycle smoke are green; the golden-set fixtures + dual-language smokes catch
  drift; the notebook gallery imports (not copies) the scene modules; C++ stays
  frozen with the retire checklist tracked.

## Retire-Later Checklist (C++ `dart-demos`)

Retire `examples/demos` (the C++ app), `pixi run demos`, and the
`run_cpp_example.py` `demos` spec only when ALL hold. Current status (Phase 5
explicit "not now"):

1. **NOT MET.** Python headless runner covers the breadth (≥ C++ pedagogical
   coverage). Today: 11 Python scenes vs 37 C++ scenes. Closing this gap is the
   Phase 3 follow-up — port the remaining C++-only categories that have no
   Python counterpart.
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
5. **MET (mechanism); EXPANDING.** Cross-language golden parity is
   implemented end-to-end for `hello_world` (Python `test_golden_parity` + C++
   `UNIT_gui_DemosGoldenParity` both pass against the shared fixture). The
   other planned golden scenes iterate on the same mechanism without
   architectural change.

Until conditions 1–3 are met, C++ `dart-demos` stays frozen-but-present.

## Open Gaps

- Colab single-source install: how notebooks in `python/tutorials/` import
  `python/examples/demos` scene modules in a managed runtime (packaged import vs
  `%pip install` from the repo subdirectory) is shared with PLAN-012.
- The `dartpy.gui` headless surface may need camera/screenshot additions for the
  Python golden screenshot floor; scope confirmed in Phase 2.
- Final golden tolerance + step count per scene are set when fixtures are
  generated in Phase 2.
- `sensor_depth_segmentation` depends on the renderer's depth/segmentation
  outputs (the fidelity-profile seam, PLAN-090) being reachable headless from
  `dartpy.gui`; confirm before Phase 3.

## Current Evidence

- Python surface today: `python/examples/{hello_world,experimental_rigid_body}`;
  `python/tutorials/` is empty; no `notebooks/`; no parity tests; `pixi run
py-ex` runs a Python example.
- `dartpy.gui` is a headless submodule (`python/dartpy/gui/module.cpp`,
  `descriptors.cpp`) registered as `m.def_submodule("gui", ...)`, alongside
  `simulation_experimental`.
- C++ `dart-demos` (PLAN-102) hosts the consolidated GUI examples; design in
  [`../design/demos_app.md`](../design/demos_app.md).
- PLAN-012 targets Colab-from-GitHub notebooks and is backend-hidden, matching
  the headless `dartpy.gui` constraint.
