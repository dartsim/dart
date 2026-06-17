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
  - [x] Tier-3: full-catalog headless *render* pass through the real Filament
        viewer (`py-demos-smoke --render`, `pixi run -e cuda py-demos-render-smoke`,
        `test_demos_render_smoke.py`). **155/155** render non-blank; blank-detector
        verified (rejects a uniform frame). Software-Mesa env applied for dev hosts.
- [ ] **M1 — Best domain + solver**: pick the flagship domain (rigid body) and
      solver, and make that path excellent end-to-end (visual + panels + replay
      + capture + docs).
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
  headless correctness (that lives in tests). py-demos owns *visual inspection*.

## Key Decisions

- **Battle-test = no-crash across the *full* catalog.** Today only the first 3
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

1. Finish the dev build (`pixi run build-py-dev-docking`) so the viewer binding
   is importable.
2. Run a full-catalog headless no-crash smoke (every registered scene id) and
   record which scenes fail today — this is the M0 ground-truth baseline.
3. Triage failures into: fix-now, quarantine (mark planned/skip), or defer.
4. Promote the full-catalog smoke into the test suite so the catalog stays green
   as it grows.

See `01-milestones.md` for the detailed milestone criteria and solver analysis.
