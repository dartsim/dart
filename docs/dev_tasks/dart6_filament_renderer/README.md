# DART 7 Filament renderer (milestone M6a)

Status (2026-10-11): M6a merged in
[#3696](https://github.com/dartsim/dart/pull/3696), merge `b63cc07f6`. Main's
GUI backend is Filament, including the repaired native FTexture use-after-free
and interim nanobind `dartpy.gui`. `CI Filament` passed on Linux, Windows and
macOS. The x86_64 stable-ABI wheel is locally qualified; the separately cached
SDK stage, aarch64/Windows/macOS wheel qualification, quiet-host performance,
and final release reviews remain open. M6b's Python render/viewer API remains
pending.

## Contract

DART 7.0 uses Filament as the sole GUI renderer; this topic is milestone M6a
of the maintainer-approved DART 7.0 plan. Decisions D30-D32 and D31a are
recorded in the [DART 7.0 project home](../dart7_python_first/README.md). There is
no 6.21 release: `main`'s 6.21 work folds into 7.0.0. Remove the OSG
component, installed GUI headers, Python namespace, dependencies, examples,
and selectors. Keep public core C++17, `dart::utils`, collision/simulation
behavior, gz-physics/gz-sim compatibility, `main`'s option defaults, and its
legacy CMake components. M6a owns the C++ backend in `dart-gui`, OSG removal,
wheel/CI packaging, and the interim nanobind GUI bindings; M6b (the DART 7.0
program) owns `dart.render`, `dart.viewer`, `dart.Material`, and deleting the
interim module before 7.0 rc.

Feature parity and release acceptance still require fresh Linux/native,
installed-consumer/wheel, platform, and performance evidence. Linux RTX 5070 Ti
and software-driver comparisons must stay within measured OSG baseline noise.
Windows/macOS performance is maintainer-deferred; native functional CI is
mandatory. User approval to replace OSG does not establish those results.

## Decisions (2026-10-10 completion session)

- The native crash was a Filament raw-handle lifetime bug in the ImGui
  overlay, not driver behavior. Filament 1.77.3 caches the raw handle of every
  single-level texture in each material instance; the overlay destroyed
  retired font-atlas textures while the base and retained clip instances still
  cached them; shared-UBO growth recommitted every instance and resent the
  stale handle. Every retired ImGui texture is now unbound from all overlay
  instances first. Source comment: `dart/gui/detail/imgui_overlay.cpp`.
- M6a (2026-10-11): the branch is a port onto a fresh `main` (not a merge of
  the old branch). `DART_BUILD_GUI_OSG=OFF` caches a headless build,
  `DART_BUILD_GUI_OSG=ON` fails with migration text, an installed `gui-osg`
  stub reports the migration and never forwards, and a missing Filament SDK,
  GLFW, libpng, or libjpeg fails configuration naming `DART_BUILD_GUI=OFF`.
  `dartpy.gui` stays the interim camelCase nanobind port (D31a); callbacks
  that cannot attach to a finalizing interpreter raise instead of inventing a
  result. The capture smoke test is opt-in (`DART_ENABLE_GUI_SMOKE_TESTS`,
  enabled by `CI Filament`). The private SDK stages Filament's pinned Vulkan
  headers so the manylinux image builds it.
- Merged `main`'s 6.21 layout with Filament in place of OSG: `dart-gui` is
  the Filament GUI plus ImGui and depends only on core; `external-imgui`
  forwards to it; `gui-osg` is gone. `DART_BUILD_GUI` defaults ON, dartpy
  (nanobind, all-in-one) requires it, and `pixi run config` builds both into
  `build/<env>/cpp/Release`. This supersedes the branch's earlier opt-in GUI
  and separate GUI tree. `DART_BUILD_GUI_OSG=OFF` still disables the GUI.
- GUI tests that need a graphics device carry the CTest label `gui-device`
  and are excluded from `pixi run test`/`test-all`; the Python `pytest` target
  excludes rendering tests. Device-free GUI tests still run. The `gui` label
  covers all GUI tests;
  `pixi run test-gui` and `CI Filament` run them.
- Upstream rewrote `main`'s history on 2026-10-10 (a purged MuJoCo comparison
  lane). The branch was rebased onto the rewritten base `ec43ea419fe` before
  merging `main`, so no purged commit is republished.
- Behavior parity fixes: default 30-degree FOV, layout/Shift-aware key
  symbols with key repeat, input dispatched only inside its own viewer's
  frame, Python viewer calls hold the GIL, GUI-optional `contact_benchmark`,
  X11 windows on Linux (XWayland on Wayland desktops, as OSG used), and
  minimized windows that keep simulating.
- Durable facts live in the visualization guide, `build.rst`, CHANGELOG,
  onboarding docs (`building.md`, `testing.md`, `ci-cd.md`, `profiling.md`),
  `docs/ai/verification.md`, and the AI skills/agents.

## Remaining gates

1. Qualify the aarch64, Windows and macOS wheels. The merged evidence records
   only the x86_64 stable-ABI wheel's repair, audit and fresh cp312/cp313 smoke
   checks; native GUI CI is already green on all three operating systems.
2. C22 headless import already passed locally on the x86_64 cp312 stable-ABI
   manylinux_2_28 wheel: no graphics libraries among the NEEDED entries of its
   14 ELF objects, import plus ten `World.step()` calls in `python:3.12-slim`,
   and the typed `RenderingUnavailableError` for capture, window and default
   window. What remains is the aarch64 wheel and the `Publish dartpy` runs
   themselves, which invoke `scripts/check_headless_wheel.py` for the Linux
   stable-ABI rows.
3. Packaging: a separately cached Filament SDK stage for x86_64 and aarch64
   in the manylinux image, keeping the wheel job under 120 minutes, with the
   existing wheel license notices retained.
4. The C2 gate (about 8 weeks before API freeze): performance re-measured on a
   quiet host (GPU and Lavapipe) with an approved fast-profile disposition;
   historical Lavapipe timings are not release evidence.
5. Final release reviews after any publication-driven changes.

## Read-only DART 7 references

- [#2687](https://github.com/dartsim/dart/pull/2687), merge
  `e289b7d24a872e5f07693f2183797c1b070dda85`, promotes Filament through a
  neutral Engine/GLFW/ImGui GUI and replaces the legacy renderer surface.
- [#3378](https://github.com/dartsim/dart/pull/3378), merge
  `73e8aa81f6d729e9b0820390e2fef9d7edaa323f`, rebinds a replacement font
  texture to the base material instance and every retained clip instance.
- [#3467](https://github.com/dartsim/dart/pull/3467) uses static Filament in
  wheels to avoid eager shared GLVND imports.

## Evidence boundary

`verification.md` records this session's qualification first, then the
historical preview packets with their original source hashes. Historical
packets qualify those old bytes only.
