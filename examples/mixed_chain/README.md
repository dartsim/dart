# Mixed Chain Example

## Summary

- Goal: visualize a mixed rigid/soft articulated chain and apply short impulses
  to one soft link.
- Concepts/APIs: SKEL loading, soft body nodes, external forces, pre-step
  callbacks, renderer-neutral keyboard actions, and public `dart::gui` panels.
- Expected output: a mixed rigid/soft chain initialized from
  `test_articulated_bodies_10bodies.skel`.
- Controls: `q`/`w`, `e`/`r`, and `t`/`y` apply short impulses in the
  negative/positive X, Y, and Z directions. Space toggles simulation.

## Run

From the source tree:

```bash
pixi run ex mixed_chain
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex mixed_chain --headless --frames 2 --screenshot /tmp/mixed_chain.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex mixed_chain --headless --frames 3 --out /tmp/mixed_chain_frames
```
