# Examples Strategy — Dev Task

Implement PLAN-103 (Python-first examples strategy): a headless Python
scene-registry runner + a notebook gallery, with a thin golden-set parity smoke,
while C++ `dart-demos` (PLAN-102) stays frozen.

- Plan: [`../../plans/103-examples-strategy.md`](../../plans/103-examples-strategy.md)
  (PLAN-103) — operating status in
  [`../../plans/dashboard.md`](../../plans/dashboard.md).
- C++ app architecture: [`../../design/demos_app.md`](../../design/demos_app.md).

## Current Status

- [x] **Phase 1** — `python/examples/demos/` headless runner + registry; 4 seed
      scenes (hello_world, sx_articulated, sx_floating_base, sx_contact); `pixi run
py-demos`; pytest cycle smoke.
- [ ] **Phase 2** — golden parity harness: 4 golden scenes
      (`hello_world`, `rigid_chain`, `boxes`, `operational_space_control`), shared
      `python/examples/demos/golden/<scene>.json` expected-state fixtures, C++ +
      Python golden smokes; extend `dartpy.gui` headless (camera/screenshot) as far
      as the screenshot floor needs.
- [ ] **Phase 3** — modernize content Python-first: `legged_whole_body_control`,
      `contact_rich_manipulation`, `rl_gym_env`, `mpc_cartpole`,
      `sensor_depth_segmentation`.
- [ ] **Phase 4** — notebook gallery in `python/tutorials/` importing the demos
      scene modules; Colab smoke (PLAN-012).
- [ ] **Phase 5** — retire C++ `dart-demos` per the checklist in PLAN-103.

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

1. Phase 2: define the golden set, write fixture-generation tools (run once,
   commit), add the Python golden smoke and a C++ golden smoke that asserts
   against the same fixtures.
2. Phase 3: implement the legged whole-body control scene first (highest
   research-value modern scenario); then RL gym env wrapper.
