# DART Compatibility Policy

## Overview

DART determines minimum compiler versions, C++ standards, and dependency versions based on Ubuntu Long Term Support (LTS) releases. This policy ensures DART can be easily built and installed on commonly used Linux distributions while leveraging modern C++ features.

## Policy

**DART supports at least two active Ubuntu LTS versions at any given time.**

All minimum requirements are determined by the **oldest supported Ubuntu LTS version**:

- Compiler versions match the default compiler in that LTS
- C++ standard is based on what those compilers fully support
- Library dependency versions match what's available in the Ubuntu repositories

## Finding Current Requirements

Rather than documenting specific version numbers here (which become outdated), refer to these authoritative sources:

### Current C++ Standard and Compiler Requirements

- **C++ Standard**: See [`cmake/dart_defs.cmake`](../../cmake/dart_defs.cmake) - search for `target_compile_features` with `cxx_std_XX`
- **Tested Platforms**: See [`CHANGELOG.md`](../../CHANGELOG.md) - check the latest release for tested compilers
- **CI Configuration**: See [`.github/workflows/`](../../.github/workflows/) for actual tested platforms

### Dependency Versions

- **CMake Requirements**: See root [`CMakeLists.txt`](../../CMakeLists.txt) - look for `find_package()` calls with version requirements
- **Managed Dependencies**: See [`pixi.toml`](../../pixi.toml) - lists all dependencies with pinned versions for reproducible builds
- **Ubuntu Packages**: See [`docs/onboarding/building.md`](building.md) for platform-specific installation commands

## Rationale

This approach provides:

1. **Predictability** - Clear policy tied to Ubuntu LTS cycles
2. **Stability** - Two-LTS support window for adoption
3. **Modernity** - Regular updates enable new C++ features
4. **Ease of Installation** - Matches widely-available Ubuntu packages
5. **Reasonable Upgrade Path** - 2+ years before dropping LTS support

## When Requirements Change

Changes occur with major DART releases and are:

1. Announced in [`CHANGELOG.md`](../../CHANGELOG.md)
2. Documented in migration guides
3. Given at least one release cycle of deprecation notice

## Questions

For questions about this policy:

- [GitHub Issues](https://github.com/dartsim/dart/issues)
- [GitHub Discussions](https://github.com/dartsim/dart/discussions)
