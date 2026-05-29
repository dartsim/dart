# DART Tutorials README

The legacy OpenSceneGraph tutorial executables were removed with the renderer
replacement. Maintained visualization coverage now lives in the Filament GUI
implementation behind `dart::gui` and the restored GUI examples.

From a source checkout, launch the standalone editor:

```bash
pixi run dartsim
```

For migrated scene fixtures such as `atlas-simbicon`, `tinkertoy`,
`mimic-pendulums`, `rigid-chain`, and contact examples, pass `--scene <name>`
through the example runner (`pixi run ex dartsim --scene <name>`). For the
current example catalog, see `examples/README.md`.
