# Verification Guide

Use the narrowest gate that proves the change, then broaden when the touched
surface affects shared behavior.

## Baseline Checks

- Always inspect `git status --short --branch`.
- Before every commit, run `pixi run lint`.
- For C++ or Python code changes, run `pixi run build` and the focused tests for
  the touched module.
- For package, exported target, installed-header, collision, constraint, or
  default-solver changes that can affect Gazebo/gz-physics, run the Gazebo gate:

  ```bash
  N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
  DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
  ```

## Docs And Workflow Changes

- For AI workflow changes, run:

  ```bash
  pixi run sync-ai-commands
  pixi run check-ai-commands
  pixi run lint
  ```

- For ordinary docs-only changes on this release branch, run `pixi run lint`.
- For docs placement, AI operating-model, plan/dashboard, or workflow-source
  changes, also run `pixi run check-ai-commands`; run
  `pixi run sync-ai-commands` first when `.claude/` sources changed.
- If the docs affect Read the Docs pages, run `pixi run docs-build` when
  practical.

## Visual Verification (Headless Capture)

DART 6 renders off-screen through the `dart::gui::osg` helper
(`dart/gui/osg/OffscreenViewer.hpp`). The GLX pbuffer path needs an X server:
use the workstation display, or wrap headless hosts in
`xvfb-run -a -s '-screen 0 1280x1024x24' <command>`. conda-forge has no
linux-64 Xvfb build, so install Xvfb from the system package manager (or run on
a host with a real X server); the capture path skips cleanly when no `DISPLAY`
is set.

- Capture a still from a GUI example:

  ```bash
  pixi run capture                                   # sleeping, 640x480 -> capture.png
  pixi run capture ssik_ik_gui arm.png
  ```

- Or capture from dartpy (release-6.20 offscreen bindings):

  ```python
  import dartpy as dart

  viewer = dart.gui.osg.ImGuiViewer()
  viewer.addWorldNode(dart.gui.osg.WorldNode(world))
  eye, center, up = dart.gui.osg.defaultAgentCamera([0, 0, 0], radius)
  viewer.captureOffscreen("out.png", eye, center, up, width=640, height=480)
  ```

- Turn a capture into a machine-readable verdict, or compare against a golden:

  ```bash
  pixi run image-verdict out.png                 # JSON: non-blank, contrast, diff
  pixi run image-verdict out.png golden.png      # diff a capture against a golden
  pixi run image-golden capture.png golden.png --update   # regenerate a golden
  pixi run image-sheet frame_0.png frame_1.png --out sheet.png --grid 3x3
  ```

The verdict JSON (`schema_version dart.image_verdict/v1`) sets `pass` from the
non-blank check plus any golden diff. Contrast is scene-dependent, so it is
reported but only gates when you pass `--require-contrast`.

For physics determinism (rather than visual appearance), use the text path that
already exists on the branch: `pixi run bm-boxes-headless` prints per-step
position/velocity checksums with no rendering, so diffing two builds' output
proves byte-identical simulation. Prefer this for logic/solver changes; use the
image verdict for anything that affects what is drawn.

## Completion Audit

Before calling work complete, verify every explicit requirement against current
evidence: files, command output, tests, PR state, and downstream gates where
applicable. If evidence is indirect or missing, keep working.
