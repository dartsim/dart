# Code Style

Follow the existing style in nearby files.

- Use two-space indentation in C++.
- Use project naming conventions already present in the touched module.
- Keep includes minimal and ordered consistently with neighboring files.
- Prefer small, behavior-preserving commits for mechanical changes.
- A class that inherits `Frame` (or any base aligned above 8 bytes) virtually
  declares `alignas(<that base>)`, and a generic virtual-inheritance helper
  such as `common::Virtual<T>` declares `alignas(T)`, so the non-virtual part
  is as aligned as the class: GCC's base-object constructors assume the full
  alignment of `this` (dartsim/dart#3447, `docs/design/dart6_frame_alignment.md`).
  `UNIT_dynamics_FrameBaseAlignment` checks a fixed list of Frame-family
  classes; add new ones to that list.

Run `pixi run lint` before committing.
