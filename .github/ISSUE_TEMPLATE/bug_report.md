---
name: Bug report
about: Create a report to help us improve
title: ""
labels: "type: bug"
assignees: ""
---

> For questions, use [Discussions](https://github.com/dartsim/dart/discussions) instead.
> For platform-specific build/test failures, use the [Build/Test Failure template](https://github.com/dartsim/dart/issues/new?template=build-test-failure.yml).

**Before submitting:**

- [ ] Checked [documentation](https://dart.readthedocs.io/) and existing issues

**Environment:**

- DART version: [e.g., main, 6.16.x]
- OS: [e.g., Ubuntu 24.04, macOS Sequoia, Windows 11]
- Installation: [e.g., pixi, conda, apt, homebrew, vcpkg, source]
- Compiler (if source): [e.g., GCC 13.2.0, Clang 18.0.0, MSVC 2022]

> **Note:** Bug fixes are only accepted for `main` and `release-6.16`. If using 6.15 or older, please upgrade to 6.16.x first and verify the issue still exists.

**Expected vs Current Behavior:**
Describe what you expected and what actually happens.

**Steps to Reproduce:**

1.
2.
3.

**Code to Reproduce:**

```cpp
// Minimal reproducible example
```

**Additional Info:**

- For build issues, share verbose configure/build output (e.g., `DART_VERBOSE=ON` and the full configure command)
- If this is distro/packaging-specific, include the packaging recipe or a minimal repro (e.g., Docker/chroot)
- Share gist links for long outputs
