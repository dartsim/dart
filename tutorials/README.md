# DART Tutorials README

The legacy OpenSceneGraph tutorial executables were removed with the renderer
replacement. Maintained visualization coverage now lives in the Filament GUI
example scenes and backend-hidden `dart::gui::experimental` APIs.

From a source checkout, run:

```bash
pixi run ex filament_gui
```

Pass `--scene <name>` for migrated scenes such as `atlas-simbicon`,
`tinkertoy`, `mimic-pendulums`, `rigid-chain`, and contact examples. For the
current example catalog, see `examples/README.md`.
