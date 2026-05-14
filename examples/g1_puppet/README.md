# G1 Puppet Example

## Summary

- Goal: load a remote URDF package and inspect a G1 model with the experimental
  Filament viewer.
- Concepts/APIs: `dart::io::readSkeleton`, `HttpResourceRetriever`,
  `PackageResourceRetriever`, and the experimental Filament GUI scene.
- Expected output: a kinematic G1 in a Filament viewer.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  items, and Escape exits.

## Notes

- Override the package/robot source with `--package-uri`, `--robot-uri`, and
  `--package-name`.
- Requires CLI11 for the legacy standalone source in this directory.
- The recommended in-tree runner now uses the Filament scene in
  `examples/filament_gui`. It exposes colored IK targets for the hands and
  feet; press `1`-`4` or click a target marker, then Ctrl-left drag or use the
  arrow/PageUp/PageDown keys to move the target and solve IK. The standalone
  source in this directory remains as the legacy OSG version.
- This example runs in kinematic mode (no physics simulation).

## Run In Tree

From the repository root:

```bash
pixi run ex g1_puppet --gui-scale 2
```

This builds and runs `examples/filament_gui --scene g1`, so it uses the same
Filament renderer path as the other experimental GUI fixtures. The legacy G1
source accepts the shorter option names; the Filament runner accepts both those
names and the explicit `--g1-*` names.

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
