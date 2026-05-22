# DART Tutorials README

The legacy OpenSceneGraph tutorial executables were removed with the renderer
replacement. Maintained visualization coverage now lives in the Filament GUI
implementation behind `dart::gui` and the restored GUI examples.

From a source checkout, run:

```bash
pixi run ex dartsim
```

Pass `--scene <name>` for migrated scenes such as `atlas-simbicon`,
`tinkertoy`, `mimic-pendulums`, `rigid-chain`, and contact examples. For the
current example catalog, see `examples/README.md`.
