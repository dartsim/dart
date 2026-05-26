# Examples Strategy — Dev Task

Implement PLAN-103 (Python-first examples strategy): a headless Python
scene-registry runner + a notebook gallery, with a thin golden-set parity smoke,
while C++ `dart-demos` (PLAN-102) stays frozen.

- Plan: [`../../plans/103-examples-strategy.md`](../../plans/103-examples-strategy.md)
  (PLAN-103) — operating status in
  [`../../plans/dashboard.md`](../../plans/dashboard.md).
- C++ app architecture: [`../../design/demos_app.md`](../../design/demos_app.md).

## Current Status

- [x] **Phase 1** — `python/examples/demos/` headless runner + registry; seed
      scenes (`hello_world`, `kr5_arm`, sx demos); `pixi run py-demos`; pytest
      cycle smoke.
- [x] **Phase 2** — golden parity harness landed. Shared
      `python/examples/demos/golden/hello_world.json` fixture; Python
      `test_golden_parity` and C++ `UNIT_gui_DemosGoldenParity` both assert
      against it within 1e-9. `examples/demos` split into a `demos_scenes`
      static library + thin `dart-demos` app so tests can link the registry.
- [x] **Phase 3** — 5 modern Python-first scenes (minimal viable, not
      research-grade): `legged_balance`, `arm_push_box`, `cartpole_gym_env`,
      `cartpole_mpc`, `sensor_descriptors`. 11 scenes total.
- [x] **Phase 4** — notebook gallery seeded: `python/tutorials/01_browse_demos.ipynb`
      imports the demo scene modules (single source). Colab publication +
      smoke owned by PLAN-012.
- [ ] **Phase 5** — retire C++ `dart-demos`: **explicit "not now"**. Conditions
      1–3 of the retire-later checklist are not met (Python breadth gap, no
      Colab smoke, no editor scene loading). Conditions 4–5 are met. See
      `docs/plans/103-examples-strategy.md`.

## Goal

Python becomes DART's primary, growing example surface (headless runner +
notebook gallery); C++ examples stay frozen at PLAN-102's 37-scene set. Common
scene + physics logic is near-identical across languages, guarded by a
golden-set parity smoke.

## Key Decisions (locked in PLAN-103)

- Python is the source of truth for common example content going forward.
- Scenes live in `python/examples/demos/` as an importable package; notebooks
  import (not copy) those modules.
- Runner CLI mirrors `dart-demos`: `--scene`, `--cycle-scenes`, `--frames`,
  `--screenshot`; always headless.
- No interactive `dartpy` viewer binding; `dartpy.gui` stays headless.
- Parity = thin golden smoke (~3-5 scenes), not 1:1 duplicate maintenance.

## Immediate Next Steps

Phases 1–4 are landed; Phase 5 is explicit "not now". Follow-ups to close the
remaining retire-later gates:

1. Grow the Python scene set toward C++ pedagogical coverage (mirror the
   categories the Python runner currently lacks).
2. Extend the golden set beyond `hello_world` (add `rigid_chain`, `boxes`,
   `operational_space_control` Python scenes mirroring their C++ counterparts +
   regenerate fixtures via `python -m examples.demos.golden._generate`).
3. PLAN-012 (Cloud Dartpy Tutorials): publish the notebook gallery with a
   green Colab smoke.
4. PLAN-101 (`dartsim` editor): wire it to open curated example scenes
   interactively. When all three follow-ups land, Phase 5 retire is unblocked.
