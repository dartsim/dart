---
name: dart-verify-sim
description: "DART Verify Sim: text-first and OSG visual debugging for DART 6 scenes, dynamics, collision, contacts, and GUI output"
---

# DART 6 Simulation Verification

Load this skill whenever a DART 6.20 claim depends on 3D structure or
behavior: model/scene loading, dynamics, collision/contact/constraints,
simulation stepping, OSG rendering, or a visual example.

**Lead with text, corroborate with images.** A rendered frame helps an agent
understand the scene and spot gross failures, but it does not prove numerical
correctness or bounded penetration. First run a text oracle, then verify the
same claim end to end through the core OSG capture/debug path.

## Owner And Evidence Contract

Read `docs/ai/verification.md`. Record:

1. a text correctness oracle (focused behavioral test, poses/velocities,
   contact trace, or `pixi run bm-boxes-headless` checksum comparison);
2. an assessed headless capture with only the debug layers needed by the
   claim; and
3. the reproduce command, view-quality result, and limitations.

If OSG capture is unavailable or visual corroboration is genuinely irrelevant,
state why and name the replacement evidence. Never use an image as the
`sole correctness oracle`.

## Visual Debugging And Capture

Use `pixi run agent-capture -- ...` for deterministic stills, turntables, or
motion sequences. Select a built-in `--scene` or an applicable
`--factory module:callable` that returns the changed `dartpy.simulation.World`.
The sidecar JSON records the camera, layers, view report, and reproduce command.

Available layers are `grid`, `world_frame`, `body_frames`, `contacts`,
`velocities`, `coms`, `inertia_boxes`, `collision_bounds`, `trajectories`, and
`labels`. They render through `dart::gui::osg::DebugOverlay`, not image-space
annotation. Use `--focus` plus `--auto-views` when framing is uncertain;
`scripts/agent_view_quality.py` rejects cropped, off-frame, too-near, too-far,
occluded, or ambiguous views using core bounds and collision raycasts.

Typical same-claim sequence for contact/debug-overlay work:

```bash
pixi run test-agent-debug-overlay
pixi run agent-capture -- --scene box_on_ground --steps 250 \
  --layers contacts body_frames collision_bounds labels --auto-views 1 \
  --out /tmp/dart-visual-evidence
pixi run image-verdict -- /tmp/dart-visual-evidence/capture_auto0.png
```

The task's first case is the text/geometry oracle for the same settled-contact
claim shown in the capture; its second checks the underlying engine overlay,
its third exercises a valid custom factory, and its fourth performs the full
`agent-capture` A/B with identical cameras and requires debug layers to change
at least 128 pixels plus visible contact-marker color pixels, then proves each
of the contacts, collision-bounds, and labels layers changes pixels on its own.
Filtered DART 6 sentinel contacts remain diagnostic sidecar evidence rather
than an A/B failure. For a custom `--factory`, replace the text case with a
focused test, pose/contact trace, or checksum comparison that exercises that
factory; do not pair a custom scene with an unrelated benchmark. Use
`pixi run bm-boxes-headless` only when the claim is specifically about that
benchmark's deterministic box trajectory.

Use `pixi run image-compose` for before/after or diff composites and
`pixi run image-sheet` for motion. `pixi run evidence-select` keeps a bounded
claim-covering artifact set; `pixi run evidence-publish` drafts the PR visual
section. Upload or PR mutations still require explicit maintainer approval.

## Environment

OSG offscreen capture needs a real X display or Xvfb. When no display is
available, keep the text evidence, record the rendering limitation, and do not
claim visual runtime coverage.
