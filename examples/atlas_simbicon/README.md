# Atlas Simbicon Example

## Summary

- Goal: run a Simbicon-style controller on the Atlas model.
- Concepts/APIs: `dart::io::readSkeleton`, example-owned gait controller
  state machines, `dart::gui::ApplicationOptions::preStep`, keyboard actions,
  and panel controls.
- Expected output: Atlas walking on the atlas ground with Simbicon control,
  perturbation keys, and an `Atlas Control` panel.
- Controls: use r to reset, a/s/d/f to push the torso, 1/2/3/4 to switch
  state machines, and the panel for gravity, harness toggles, reset, stride
  mode, pause, step, and exit. `--gui-scale` adjusts the GUI scale, and
  `--width`/`--height` override the default 1280x960 window size.
- Remaining renderer-neutral API gaps: the historical OSG headlight toggle,
  shadow toggle, and depth camera mode need public render-settings APIs before
  exact panel parity can be restored.

## Run

From the source tree:

```bash
pixi run ex atlas_simbicon
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex atlas_simbicon --headless --frames 2 --screenshot /tmp/atlas_simbicon.ppm
```
