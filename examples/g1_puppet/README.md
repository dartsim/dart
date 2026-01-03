# G1 Puppet Example

## Summary

- Goal: load a remote URDF package and inspect a G1 model with IK handles.
- Concepts/APIs: `dart::io::readSkeleton`, `HttpResourceRetriever`,
  `PackageResourceRetriever`.
- Expected output: a kinematic G1 in an OSG viewer; keys 1-4 toggle IK targets.
- Controls: drag body nodes with the mouse; press 1-4 to toggle IK targets.

## Notes

- Override the package/robot source with `--package-uri`, `--robot-uri`, and
  `--package-name`.
- Requires CLI11 for argument parsing (installed via pixi).
- This example runs in kinematic mode (no physics simulation).

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
