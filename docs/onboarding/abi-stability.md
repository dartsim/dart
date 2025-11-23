# ABI Stability

This note tracks the work for [issue #1026](https://github.com/dartsim/dart/issues/1026): avoiding ABI collisions between different DART versions that may be loaded into the same process (common in Gazebo plugin setups).

## Current status

- **Relevant**: Still open; DART exports unversioned C++ symbols and installs headers without versioned prefixes, so different minor releases can collide at runtime.
- **SONAME**: Encodes major + minor (`SOVERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}"`), which prevents side-by-side minor installs.
- **Namespaces**: Public code lives in `namespace dart` (and `namespace dart8` for the experimental API) with no inline namespace keyed to the major version.
- **ABI padding**: A few structs (e.g., `dart/constraint/ContactSurface.hpp`) include padding fields, but most public classes do not have ABI guards.

## Reproduction of collisions

One concrete ABI break was introduced in `68aef6b6d66d3f032971752b3f30c77aa13b3c7c` (templating `createShapeNode`). To see the collision:

1. Build and install two nearby minor versions (example: `v6.3.0` and `v6.4.0`) to different prefixes using the pixi workflow:

   ```bash
   git checkout v6.3.0
   pixi run config -B build/abi-630 -DCMAKE_INSTALL_PREFIX=$PWD/install-630
   pixi run build -C build/abi-630
   cmake --install build/abi-630

   git checkout v6.4.0
   pixi run config -B build/abi-640 -DCMAKE_INSTALL_PREFIX=$PWD/install-640
   pixi run build -C build/abi-640
   cmake --install build/abi-640
   ```

2. Build two trivial plugins (copy `examples/hello_world`) linking against `install-630` and `install-640` respectively.
3. Write a tiny host that `dlopen`s both with `RTLD_GLOBAL` and calls into each plugin. The second load resolves symbols from the first DART build, leading to `undefined symbol`/crash because the mangled names are identical but the ABI differs.
4. Adding `LD_DEBUG=bindings` when running the host shows the wrong symbols being bound across the two plugin DSOs.

## What remains to fix

- **Policy**: Commit to “no ABI breaks within a major release” and document it. If a break is unavoidable, bump the major version.
- **Inline namespaces**: Wrap exported DART symbols in an inline namespace keyed to the major version (e.g., `dart::inline dart7`). This keeps source compatibility (`dart::Foo` still works) while preventing cross-major symbol collisions.
- **Install layout**: Move to versioned include directories and change the SONAME to major-only once ABI stability is enforced, so minor releases can be installed side-by-side.
- **CI guardrails**: Add an ABI check job (e.g., libabigail/abi-compliance-checker) comparing `main` against the previous minor release to catch accidental ABI breaks.
- **Bindings/tooling**: Update CMake package config, pkg-config files, Python bindings, and any `dart8` exports to follow the namespace and install layout.

## Next steps (proposed implementation order)

1. Add a central header and macros for the inline namespace name and start migrating public headers/sources to use them.
2. Switch SONAME to major-only and introduce versioned include/install prefixes with compatibility forwarding headers where feasible.
3. Land CI ABI checking against the last released minor build.
4. Update docs/migration notes and close the GitHub issue once the above lands.
