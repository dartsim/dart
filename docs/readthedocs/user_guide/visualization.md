# Visualization

Numbers in a loop are great for tests; for everything else you want to _see_ the
simulation. DART 7 ships an interactive viewer and a curated set of demo scenes,
and it can render frames headlessly for screenshots and CI.

## Run the interactive demos

From a source checkout, the demo runner opens scenes in the viewer:

```bash
pixi run py-demos                                   # open the default scene
pixi run py-demos -- --scene rigid_body             # pick a scene by id
pixi run py-demos -- --list                         # list every scene
```

Each scene builds a real `World`, steps it live, and draws the result, with
on-screen panels for tweaking parameters. Browsing the
[scene sources](https://github.com/dartsim/dart/tree/main/python/examples/demos/scenes)
is one of the fastest ways to learn idiomatic DART 7 — they use exactly the API
this guide describes.

## Render headlessly

For screenshots, documentation, or automated checks, render without opening a
window:

```bash
# One headless frame — a quick smoke check.
pixi run py-demos -- --scene rigid_body --headless --frames 1

# Capture a PNG (the helper rejects blank frames and prints the artifact path).
pixi run py-demo-capture -- --scene rigid_body --frames 2 --width 640 --height 360
```

Headless rendering uses the same render path as the interactive viewer, so a
capture is faithful evidence of what the scene looks like.

## The standalone viewer

DART also provides a standalone viewer application for source builds:

```bash
pixi run dartsim
```

## How visuals relate to physics

DART keeps **physics** and **visuals** as separate concerns: the world owns the
dynamics, and the viewer reads each body's world transform to draw it. That
separation is why a headless physics-only script (like {doc}`Hello, DART
<getting_started/hello_dart>`) needs no graphics at all, while the same world can
be handed to the viewer when you want to watch it. You can always inspect a body
visually and verify it numerically with `body.translation` — the two agree
because they read the same state.

## Next

Ready to go deeper? See {doc}`next_steps` for advanced topics, runnable examples,
and the API reference.
