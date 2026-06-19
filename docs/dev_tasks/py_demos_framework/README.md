# py-demos Framework — Dev Task

Harden `pixi run -e cuda py-demos` into a stable, scalable, modern harness for
**visually debugging and inspecting DART 7 features**, then grow its feature
coverage deliberately — strong minimal core first, feature-rich one capability
at a time, each step backed by tests and verification.

Code lives in `python/examples/demos/` (runner, registry, scenes). The
interactive viewer itself is C++ `dart::gui::run_demos` (Filament + ImGui); the
Python side supplies the scene catalog, per-step hooks, panels, debug overlays,
and replay metadata.

## Current Status

- [x] **M0 — Establish the framework**: stable, no-crash, scalable, modern. **Done** —
      full catalog green across build/step/providers, panels, and real render.
  - [x] Full-catalog no-crash baseline: **155/155** (`02-m0-baseline.md`).
  - [x] No-crash guard locked into CI: `scripts/py_demos_smoke.py` (isolated
        diagnostic) + `pixi run -e cuda py-demos-smoke` + an in-process pytest
        `python/tests/integration/test_demos_full_catalog_smoke.py` (every-PR
        guard) + unit tests for the smoke orchestrator. All green.
  - [x] Tier-2: `ScenePanel.build` coverage via a faithful fake builder
        (`python/examples/demos/_smoke_support.py`), wired into both the smoke
        script and the pytest guard. 155/155 green; verified genuine (153/155
        scenes have panels; panel code demonstrably calls the builder).
  - [x] Scalable-contract guard: `test_every_scene_module_is_registered`
        (no orphan `scenes/*.py`) complements `test_registry_has_scenes`
        (unique ids + field contract). Adding a scene is now drift-safe.
  - [x] Tier-3: full-catalog headless _render_ pass through the real Filament
        viewer (`py-demos-smoke --render`, `pixi run -e cuda py-demos-render-smoke`,
        `test_demos_render_smoke.py`). **155/155** render non-blank; blank-detector
        verified (rejects a uniform frame). Software-Mesa env applied for dev hosts.
  - [x] Interactive selection/force-drag overlay hardening: a zero-motion
        click/drag on `rigid_body` no longer feeds degenerate debug primitives
        into Filament's AABB precondition path.
  - [x] Local full-catalog regression pass on PR #3084 local head `bd0b8e7`:
        default `pixi run py-demos-smoke --json-out
/tmp/py_demos_smoke_default_pr3084_local.json` passed **155/155** in
        `real 47.96s`; CUDA `pixi run -e cuda py-demos-smoke --json-out
/tmp/py_demos_smoke_cuda_pr3084_local.json` passed **155/155** in
        `real 76.18s`.
- [ ] **M1 — Best domain + solver**: pick the flagship domain (rigid body) and
      solver, and make that path excellent end-to-end (visual + panels + replay + capture + docs). **In progress**.
  - [x] First contact-baseline increment: the flagship `rigid_body` scene
        exposes `contact_solver_method` in its panel, replay state, and capture
        metadata so Sequential Impulse and the DART 6-style boxed-LCP contact
        path can be inspected from the live demo. The panel also has one-click
        baseline presets for the default Sequential Impulse and boxed-LCP
        contact paths, plus a route into the side-by-side
        `rigid_contact_solver_compare` row. Realtime rigid workflow rows keep
        the rigid-body solver fixed to the Sequential Impulse path; use
        `rigid_solver_compare` and the explicit IPC shelf scenes for SI-vs-IPC
        visual inspection.
  - [x] Scriptable contact-policy capture: `py-demo-capture` can restore
        scene-owned replay state before launch via `--scene-state-json`, and
        `--capture-label` gives stateful A/B captures distinct artifact stems
        and default output directories. The `rigid_body` workflow panel now
        advertises both the boxed-LCP baseline live command and capture command,
        and state overrides honor explicit `--scene`, `DART_DEMOS_SCENE`, and
        the default `rigid_body` front-door selector. Verified in default and
        CUDA environments with manifests resolving
        `contact_solver_method=BOXED_LCP`.
  - [x] Dedicated contact-baseline packet: the `--contact-baseline-only`
        workflow captures the `rigid_body` Sequential Impulse and boxed-LCP
        contact-policy variants as one review-indexed packet, with per-row
        state/label metadata and rerun commands.
  - [x] AVBD rigid-constraint showcase packet: the dedicated
        `--avbd-showcase-only` workflow packet now captures the curated
        fixed-joint/contact, breakable-joint, spherical breakable-joint,
        revolute-motor, and prismatic-motor AVBD rows as a complementary M1
        packet. The manifest and review index label these rows as
        `avbd_constraint_showcase` and keep the scope explicit: this is the
        AVBD `sx` rigid-constraint track beside the World contact-policy /
        boxed-LCP baseline, not a head-to-head solver enum comparison. Verified
        with default and CUDA packet row captures.
  - [x] Reviewer-facing visual packet pass: full contact-baseline and AVBD
        showcase packets completed in both default and CUDA Pixi environments.
        The AVBD rows are visually legible with docked diagnostics; the
        contact-baseline rows are solver-evidence-first because SI and boxed-LCP
        look similar in the docked view, so labels, restored state, solver
        identity, and metrics in the review index are the key evidence. A
        unified comparison scene is not required for the current PR evidence;
        keep it as a follow-up only if review finds the two packets insufficient.
- [ ] **Beyond M1**: repeat M1 outward — expand solvers and domains
      incrementally, each with sufficient testing and verification.

## Goal

A developer/user can run `pixi run -e cuda py-demos`, browse every registered
DART 7 capability, and visually inspect/debug it without crashes, with clear
per-scene diagnostics. Adding a new capability scene is a clean, tested,
repeatable operation.

## Non-Goals (early phases)

- Rewriting the C++ viewer (`dart::gui`). We improve the Python harness and its
  contract with the existing viewer.
- Chasing every solver/domain at once. M1 commits to one flagship pair; breadth
  comes only after the core is proven stable.
- Performance benchmarking (that lives in the headless benchmark harness) and
  headless correctness (that lives in tests). py-demos owns _visual inspection_.

## Key Decisions

- **Battle-test = no-crash across the _full_ catalog.** Today only the first 3
  scenes are smoke-tested via the GUI cycle (`test_demos_cycle.py`); ~140 scenes
  are registered. M0's foundation is a full-catalog headless smoke that exercises
  every scene's build + step + render and fails loudly on any regression.
- **Code is source of truth.** No hardcoded scene lists in docs; the registry
  (`python/examples/demos/registry.py`) is the catalog.
- **M1 solver (CONFIRMED): AVBD as the modern flagship, validated against the
  DART 6 boxed-LCP solver as the trusted reference baseline.** IPC reserved as
  the accuracy / no-penetration specialist for a later milestone. See
  `01-milestones.md` for rationale.

## Immediate Next Steps

1. Keep the interactive selection repro in the validation set when touching
   `dart::gui` debug overlays:
   `pixi run py-demos -- --scene rigid_body --headless --frames 4 --width 640 --height 480 --screenshot /tmp/rigid_body.ppm --scripted-force-drag 1:sphere_0_visual:0,0,0:2`.
2. Verify the M1 contact-baseline increment with focused C++/dartpy tests,
   `python/tests/unit/test_py_demo_panels.py`, and the M0 smoke guards.
3. Use the dedicated contact-baseline packet for reviewer-facing SI/boxed-LCP
   evidence when refreshing artifacts:
   `pixi run py-demo-capture -- --rigid-workflow --contact-baseline-only --output-dir /tmp/dart_capture_rigid_contact_baseline`.
4. Use the dedicated AVBD showcase packet for reviewer-facing M1 evidence when
   refreshing artifacts:
   `pixi run py-demo-capture -- --rigid-workflow --avbd-showcase-only --output-dir /tmp/dart_capture_rigid_avbd_showcase`.
   M1 remains two complementary showcases for now: World contact-policy /
   boxed-LCP baseline packets plus the AVBD `sx` rigid-constraint showcase.
   The full-packet pass produced enough current PR evidence; revisit a unified
   comparison scene only if review asks for a stronger single-scene visual.

See `01-milestones.md` for the detailed milestone criteria and solver analysis.
