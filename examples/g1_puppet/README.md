# G1 Puppet Example

## Summary

- Goal: load a remote URDF package and inspect a Unitree G1 model with the
  promoted `dart::gui` application runner.
- Concepts/APIs: `dart::io::readSkeleton`, `HttpResourceRetriever`,
  `PackageResourceRetriever`, `dart::gui::ApplicationOptions`, promoted panel
  callbacks, public `dart::gui::Gizmo` target affordances, and source-owned
  support debug line geometry.
- Expected output: a kinematic G1 in the official Filament/GLFW/ImGui renderer
  with hand/foot IK targets, a ground grid, and a support-polygon overlay.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, press `1`-`4`
  to toggle/select targets, left-drag active gizmo handles, arrow/PageUp/PageDown
  nudges selected targets, and Escape exits.

## Notes

- Override the package/robot source with `--package-uri`, `--robot-uri`, and
  `--package-name`. The promoted runner also accepts the explicit
  `--g1-package-uri`, `--g1-robot-uri`, and `--g1-package-name` aliases.
- This example runs in kinematic mode: the world keeps historical gravity for
  scene parity, while the G1 body nodes ignore gravity and collision so IK
  manipulation is stable.
- Press `1`-`4` to toggle IK targets for the hands and feet. Active targets are
  visible as public gizmos; move or rotate a selected target to solve the
  corresponding IK chain.
- The historical OpenSceneGraph body-node drag tools do not have a full
  renderer-neutral articulated-body manipulator yet. The promoted replacement
  supports selecting and dragging the free-root renderable plus explicit public
  IK target gizmos.

## Run In Tree

From the repository root:

```bash
pixi run ex g1_puppet --gui-scale 2
```

## Build

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Run

From the build directory:

```bash
./g1_puppet
```
