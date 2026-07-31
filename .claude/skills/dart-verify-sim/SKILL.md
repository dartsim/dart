---
name: dart-verify-sim
description: "DART Verify Sim: text-first and OSG visual debugging for DART 6 scenes, dynamics, collision, contacts, and GUI output"
---

# DART 6 Simulation Verification

Load this skill whenever a DART 6.20 claim depends on 3D structure or
behavior: model/scene loading, dynamics, collision/contact/constraints,
simulation stepping, OSG rendering, or a visual example.

**Lead with text, corroborate with images.** An image-capable agent can inspect
a rendered frame, but pixels do not expose solver state and machine pixel
checks are not semantic inspection. First run a text oracle, then verify the
same claim end to end through the core OSG capture/debug path.

## Owner And Evidence Contract

Read `docs/ai/verification.md`. Record:

1. a text correctness oracle (focused behavioral test, poses/velocities,
   contact trace, or `pixi run bm-boxes-headless` checksum comparison);
2. an assessed headless capture with only the debug layers needed by the
   claim; and
3. an actual semantic inspection of the selected still or temporal frames; and
4. the reproduce command, view-quality result, reconciliation, verdict, claim
   boundary, and limitations.

If OSG capture is unavailable or visual corroboration is genuinely irrelevant,
state why and name the replacement evidence. Never use an image as the
`sole correctness oracle`.

## Image-capable Review Loop

GPT-5.6 Sol Max supports native image input and original-detail inspection.
Keep this loop capability-based so a future `dart-model-upgrade` audit can
replace the target-specific note without cloning the skill.

1. State one claim, its expected visible observation, and the text oracle that
   decides correctness.
2. Capture one assessed, claim-tied view first. Add only the debug layers needed
   by the claim; use paired plain/debug views when overlays obscure geometry.
   For turntable or motion output, inspect at least the sidecar's
   start/middle/end targets and add intervening frames or a sheet when a
   transient event matters.
3. Run `image-verdict` for artifact integrity, then open the selected PNG with
   the active agent's native image viewer. Use original detail for fine
   contacts, labels, bounds, or frame axes. A passing view report or
   `image-verdict` is not semantic visual review.
4. Record the visible observation separately from the text result. If they
   disagree, do not average them into a pass: report fail or uncertain, inspect
   the capture sidecar, reframe or recapture, and investigate simulation state.
5. Close with pass/fail/uncertain, artifact path, camera/layers, reproduce
   command, what the image shows, and what it does not prove. If native image
   review is unavailable, hand the selected files and sidecar to an
   image-capable reviewer with `pixi run verification-bundle -- ...` and
   record that limitation.

## Visual Debugging And Capture

Use `pixi run agent-capture -- ...` for deterministic stills, turntables, or
motion sequences. Select a built-in `--scene` or an applicable
`--factory module:callable` that returns the changed `dartpy.simulation.World`.
The v2 sidecar records the camera, layers, view report, reproduce command, and
deterministic static or start/middle/end inspection targets.

Available layers are `grid`, `world_frame`, `body_frames`, `contacts`,
`velocities`, `coms`, `inertia_boxes`, `collision_bounds`, `trajectories`, and
`labels`. They render through `dart::gui::osg::DebugOverlay`, not image-space
annotation. Use `--focus` plus `--auto-views` when framing is uncertain;
`scripts/agent_view_quality.py` rejects cropped, off-frame, too-near, too-far,
occluded, ambiguous, or no-bounded-renderable views using core bounds and
collision raycasts. Its framing distance fits the limiting horizontal or
vertical field of view for the requested viewport, including portrait output.

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
`pixi run image-sheet` for motion. `image-verdict` checks pixel integrity and
optional reference differences; it does not inspect meaning.
`pixi run evidence-select` keeps a bounded claim-covering artifact set.
`pixi run evidence-publish` requires the text oracle, visible observation,
text/image reconciliation, semantic verdict, and at least one explicit
not-proven boundary before it marks the PR visual section ready. Upload or PR
mutations still require explicit maintainer approval.
`pixi run verification-bundle` packages primary text evidence, a still, and an
optional grid with hashes and a semantic-review prompt for an image-capable
handoff; producing the bundle does not itself complete semantic review.

## Environment

OSG offscreen capture needs a real X display or Xvfb. When no display is
available, keep the text evidence, record the rendering limitation, and do not
claim visual runtime coverage.
