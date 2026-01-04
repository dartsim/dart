# Unified Loading Example

## Summary

- Goal: show unified loading with shared `dart::io::ReadOptions` for worlds and
  skeletons.
- Concepts/APIs: `dart::io::readWorld`, `dart::io::readSkeleton`,
  `dart::io::ReadOptions`.
- Expected output: a short summary of loaded assets printed to the console.
- Controls: CLI flags to override URIs, format inference, SDF root joint type,
  and URDF package mappings.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

Pass `--help` to see available options.
