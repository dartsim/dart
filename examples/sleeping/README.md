# Sleeping (automatic body deactivation)

Visual demo of DART's automatic body deactivation ("sleeping") and the
solver-island partition.

Several separated box stacks each form an independent **solver island**. Each
island is drawn in its own color. When an island settles and goes to sleep
(deactivates), it is tinted to a dim, cool shade — at that point its constraint
solve, forward dynamics (gravity), and position integration are all skipped each
step until something disturbs it.

## Controls

| Key | Action |
|-----|--------|
| `f` | Shoot a sphere horizontally at the next stack — it flies in and wakes the sleeping island on impact |
| `d` | Drop a box from above onto the next stack — it wakes the island it lands on, and merges two islands into one (same color) if it bridges them |
| `t` | Toggle automatic deactivation on/off (with it off, nothing ever dims) |
| `s` | Print the number of awake vs. asleep skeletons |

A yellow arc shows the predicted parabolic path of the next sphere you fire, so
you can see roughly where it will land (it is only an estimate — gravity and the
collision itself make the real impact differ).

Projectiles you fire/drop stay bright orange so you can track them; the stacks
take island colors and dim when asleep.

## What to watch

- Each stack starts in its own bright color and, after settling, fades to the
  dim "asleep" tint.
- Toggling the feature off (`t`) keeps every island bright — that is the
  baseline cost (every island solved every step). Toggling it back on lets them
  sleep again.
- Dropping a box (`d`) onto a sleeping stack re-brightens it (wake), and if the
  box bridges two stacks they take on a single color (the islands merged).

## Command-line options

| Option | Description |
|--------|-------------|
| `--gui-scale <f>` | Scale the on-screen panel for HiDPI/4K displays (e.g. `--gui-scale 2.0`). Default `1.0`. |
| `--headless` | Render a single frame off-screen to a PNG and exit (no window). Useful as a smoke test. |
| `--shot <path>` | Output PNG path for `--headless` (default `sleeping.png`). |
| `--steps <n>` | Simulation steps to settle the scene before the headless capture (default `1500`). |
| `--width <w> --height <h>` | Render size (default `1280x800`). |

The headless mode steps the world directly (independent of the real-time
clock), so the captured frame deterministically shows the settled, sleeping
state.

## Off-screen capture and Xvfb

`--headless` renders through the shared `dart::gui::osg` off-screen helper
([`dart/gui/osg/OffscreenViewer.hpp`](../../dart/gui/osg/OffscreenViewer.hpp)):
`setUpOffscreenViewer()` attaches a GLX pbuffer to the viewer and realizes it,
`captureOffscreen()` is one-shot sugar for a single still, and
`defaultAgentCamera()` frames the scene bounding sphere from a canonical 3/4
view. The same helper backs the `ssik_ik_gui` example.

The pbuffer path needs an X server. On a workstation with a display it works as
shown above. On a headless host (CI, a container, or an SSH session with no
`DISPLAY`) wrap the command in `xvfb-run`:

    xvfb-run -a -s '-screen 0 1280x1024x24' ./sleeping --headless --shot out.png

If no X server is reachable, the helper logs this same `xvfb-run` hint and the
example exits non-zero rather than producing a blank image.

From a checkout you can build-and-capture in one step with the pixi task
(prefix with `xvfb-run` on a headless host):

    pixi run capture                                    # sleeping, 640x480 -> capture.png
    pixi run capture example=ssik_ik_gui out=arm.png    # another example

The opt-in `offscreen_capture_smoke` ctest (enable with
`-DDART_ENABLE_GUI_OSG_SMOKE_TESTS=ON`) exercises this helper and asserts the
captured PNG is non-blank; it skips cleanly when no `DISPLAY` is present.
