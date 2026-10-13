# Decisions

The program-level decision is D31b in the
[DART 7 decisions](../dart7_python_first/decisions.md). It was taken by the
maintainer on 2026-10-11, together with these four choices:

- the host lives in private C++ inside dartpy;
- heavy logic goes in C++ and glue in Python;
- the bindings are `dartpy.viewer`, in the 7.0 style;
- docking ImGui is used everywhere.

The local decisions below follow from D31b. Each one rests on read-only
research, an adversarial design review, and an independent Codex review of the
plan, all at `9e4bb4e6f`. Revisit a decision only with new evidence.

## L1. Host placement: `dart/gui/app/`, target `dart-gui-app`

The host files and their namespace move as one static target that is never
installed.

- **Why here.** `dart/` is configured before `tests/` and `python/`, so both
  can test `if(TARGET dart-gui-app)`. Only top-level `dart/gui/*.hpp` headers
  are installed, so the subdirectory stays private without extra rules. The
  docking check sits next to the ImGui provider.
- **Rejected:** `python/native/viewer/`, which is configured after `tests/`
  and needs another checker exemption.
- **Revisit** if M6b needs a viewer core that also builds without docking
  ImGui.

## L2. GUI-free models: `python/native/demo_models/`, target `dart-demo-models`

`ContactArrowLayout`, SIMBICON, the soft-foot model, the adaptive soft
contact model and the soft worm model move as a static library. The library
builds with the GUI off.

- **Why.** `test_ConstraintSolver` compiles these sources in the GUI-off
  FreeBSD, coverage and ASan lanes.
- **IO gate.** Sources that need IO are added only when `dart-utils` and
  `dart-utils-urdf` exist. The embed build turns IO off.
- **Namespace.** `dart_demos` is kept, so the tests change only their
  includes.

## L3. Panels are a declarative C++ control model

Python builds `Control` trees, and C++ draws them, validates their values and
dispatches changes between frames.

- **Why.** The goal requires the GUI to be in C++.
- **Sizing.** The model covers the measured `ImGui::` inventory of the
  scenes: text, button, checkbox, slider, input, vec3, choice, color, swatch,
  plot, group and separator.
- **No format strings.** `decimals` replaces printf formats, removing a
  format-string trust boundary.

## L4. The CLI keeps the C++ flags

`runCommandLine` parses the C++ app's flags, unchanged, with strict bounded
integers. Python passes `sys.argv`.

- **Why.** Recipes in the docs, the CI and the deformable-body work already
  use these flags.
- **Strict cycle.** `--cycle-scenes` exits nonzero when any scene callback was
  disabled, so broken ports fail CI.

## L5. `human_joint_limits` is dropped

The scene needs tiny-dnn. No Pixi environment, lock file or wheel image
provides it, so the scene has never built in CI.

- **Cost of keeping it.** The weight files are cereal binary
  (6/8→128→128→128→1 with tanh and sigmoid; about 34k parameters each). An
  Eigen reader would be about 80 lines. Porting the two `ConstraintBase`
  subclasses (about 1,250 lines) with no reference oracle is the expensive
  part.
- **Follow-up.** `data/humanJointLimits` is deleted in WP-PD.18; the format
  notes above are the follow-up record.

## L6. The two Python-only scenes are deleted

`hello_world_gui` duplicates `hello_world` and the KR5 scenes.
`contacts_pointcloud` is covered by the host's contact visualizer and by
`point_cloud`. Nothing outside the catalog references either scene.

## L7. The WAM IKFast solver is linked in

`tests/integration/SharedLibraryWamIkFast.*` moves into `dart-demo-models` as
`WamIkFast`. It is compiled without `IKFAST_CLIBRARY`, so no symbols are
exported.

- **No LAPACK dependency.** The generated code only declares the LAPACK
  routines and never calls them. `nm` of the existing shared builds shows no
  LAPACK symbols, and each is about 175 KB.
- **Rejected:** a dlopen'd library behind a new public `SharedLibraryIkFast`
  binding, which would add path discovery and would still miss wheels.

## L8. ssik becomes a Python scene

The C++ bridge embedded CPython to call ssik's Python solver. The Python scene
calls ssik directly, through DART's native `setPythonAnalytical`.

- `SsikBridge` and `ArmSkeleton` are deleted.
- The standalone example merges into the catalog. Its self-test becomes a
  pytest.

## L9. The memory-diagnostics option keeps its name

`DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS` stays off by default and moves to
`dart/gui/app/`. Its compile definition is PUBLIC because it changes `App`'s
layout. The compiled-out marker check runs on the dartpy module.

## L10. `--debug-print-world` is the parity oracle

This hidden flag prints construction counts after the headless steps. Both
apps share the host, so the counts compare directly. The flag is deleted with
the C++ app.

- **Rejected:** a full state digest. The ports build worlds differently and
  the random scenes use different generators, so the digests could not match.

## L11. `DART_IMGUI_DOCKING` is removed

When the option is set, configure prints a status line and unsets the cached
value. Removing it avoids the stale-cache trap, and no checker change is
needed.

## L12. Callback errors and interrupts

- **Ordinary exceptions.** A Python `Exception` becomes a C++ error that the
  host's existing guards log and count, and the failing callback is disabled.
- **KeyboardInterrupt and SystemExit.** They are stashed, stop the loop, and
  are re-raised after an orderly teardown.
- **Why both paths.** The host stays independent of nanobind, while the
  strict cycle still sees Python failures.

## L13. Packets land as one PR each, and the C++ app stays until parity

The C++ app is removed only after the cutover packet, so every scene's parity
is checked against the running C++ app.
