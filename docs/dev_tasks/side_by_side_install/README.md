# Side-by-Side Installation (Major+Minor) (Issue #1026)

## Status

- Planning; scope defined for major+minor version coexistence.

## Problem Statement

Downstream plugin ecosystems may need to install and build against multiple
major and minor DART versions concurrently. Current install layout and package
metadata assume a single installed version, which leads to file conflicts.

## Scope

- Allow parallel installation of multiple major+minor versions.
- Cover all install artifacts: headers, libraries, CMake packages, pkg-config,
  and runtime data.
- Maintain clear, discoverable package selection for downstream builds.

## Out of Scope

- Inline namespaces or symbol versioning (runtime mixing still unsafe).
- ABI stability guarantees across minor releases.
- Cross-distro packaging policy changes (handled by packagers).

## Goals

- Install different major+minor versions without file collisions.
- Make it explicit which version a downstream is using at configure time.
- Keep the default workflow simple for single-version users.

## Plan

- Define install directory layout for versioned headers and data.
- Version CMake package namespace/config location and target names.
- Version pkg-config file names and their include/lib paths.
- Ensure runtime data (e.g., shared resources) is versioned if needed.
- Provide migration guidance for downstreams (Gazebo/gz-physics).

## Implementation Notes

- Headers: versioned include prefix (e.g., `include/dart-<MAJOR>.<MINOR>/`).
- CMake: install config under a versioned directory and expose versioned
  package names (e.g., `DART<MAJOR>_<MINOR>`).
- Pkg-config: versioned `.pc` files (e.g., `dart-<MAJOR>.<MINOR>.pc`).
- Libraries: SONAME already includes major.minor; avoid conflicts in libdir
  by versioned filenames and/or subdirs if needed.

## Deliverables

- Updated CMake install rules to place versioned artifacts.
- Updated CMake package config to find versioned installs.
- Updated pkg-config generation to emit versioned names/paths.
- Documentation for downstream selection and migration.

## Validation / Success Criteria

- Two different major+minor versions can be installed in the same prefix.
- `find_package` and pkg-config can select the intended version unambiguously.
- Downstream builds (Gazebo) can pin a specific major+minor version.

## Risks / Trade-offs

- Increased verbosity for downstreams specifying versioned packages.
- Potential friction for existing users and build scripts.
- Still does not enable runtime mixing of multiple versions in one process.

## Open Questions

- Do we provide unversioned aliases for the latest installed version?
- Should versioned installs live in subdirs or use versioned filenames?
- How to handle header-only includes that expect `#include <dart/...>`?
