# Resume Instructions

Current phase: Phase 2 (scene ports) — dispatch batches per BRIEF-phase2.md.
IMPORTANT: batches share build/default/cpp/Release — run implementation
workers SEQUENTIALLY (build-dir contention corrupts parallel builds; a
load-84 pileup already happened once). Review every batch's PNGs before
accepting.

1. Read `README.md` (scope, decisions) and `PLAN.md` (phases, acceptance
   criteria, current checkboxes).
2. Check `git log --oneline origin/release-6.20..HEAD` on the current topic
   branch for landed work.
3. Continue at the first unchecked item of the earliest incomplete phase in
   `PLAN.md`.

Session log:

- 2026-07-04: task folder created on `feature/dart6-consolidated-demos`; recon
  agents dispatched (examples inventory, gui/osg capabilities, DART 7 demos
  architecture + history mining). PLAN.md drafted;
  `EVIDENCE-gui-capabilities.md` written.
- 2026-07-04: verification harness validated: `pixi run build` green on
  release-6.20; rebuilt `sleeping` and captured a headless PNG
  (`--headless --shot`, DISPLAY=:0 + pbuffer) showing scene + ImGui panel.
  Note: `build/default/cpp/Release/bin` contains STALE binaries from an old
  main checkout (`dart-demos`, `dartsim`, `island_deactivation`, ...) — always
  rebuild a target before trusting it. Removed stale
  `python/examples/demos/__pycache__` leftovers (untracked).
