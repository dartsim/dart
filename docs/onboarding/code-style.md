# Code Style

Follow the existing style in nearby files.

- Use two-space indentation in C++.
- Use project naming conventions already present in the touched module.
- Keep includes minimal and ordered consistently with neighboring files.
- Prefer small, behavior-preserving commits for mechanical changes.
- Windows CI and the Linux assertions gate (`Asserts enabled (no -DNDEBUG)`)
  compile up to 8 source files of a target as one translation unit (a CMake
  unity build). So keep names local to a `.cpp` file unique within its
  target: anonymous-namespace or `static` helpers, constants, and types. Also
  `#undef` any macro a `.cpp` file defines, at the end of that file. The
  nightly `Unity name clashes` job compiles each target as a single unity
  file, so it reports a clash before a source-list change exposes it.
- A class that inherits `Frame` (or any base aligned above 8 bytes) virtually
  declares `alignas(<that base>)`, and a generic virtual-inheritance helper
  such as `common::Virtual<T>` declares `alignas(T)`, so the non-virtual part
  is as aligned as the class: GCC's base-object constructors assume the full
  alignment of `this` (dartsim/dart#3447; see the comments at those
  declarations). `UNIT_dynamics_FrameBaseAlignment` checks a fixed list of
  Frame-family classes; add new ones to that list.

Run `pixi run lint` before committing.
