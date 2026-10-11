# Verification

## DART 7 M6a port (2026-10-11, branch `feature/dart7-filament-m6a`)

Upstream rewrote `main`'s history on 2026-10-10 (a purged MuJoCo comparison
lane). The Filament checkpoint was replayed onto the rewritten base
`ec43ea419fe`; its tree matched the original except for the purge. The
replayed commit kept a session scratch path in this file, which `main`'s new
`check_local_paths.py --commit-range` rejects, so the port source is the
sanitized equivalent `fc4a3d9cc31`. `main` (23 commits: nanobind-only dartpy
and pybind11 removal, the 6.21 core/IO/GUI consolidation, nanobind OSG
bindings, local-path checks, manylinux_2_28 wheels, retired and new demos)
was merged into it, all 68 conflicts were resolved, and that merged tree was
applied onto `main` `3d31d568794`. The result equals the merged tree plus
`main`'s three newer commits.

- Merge resolution:
  - `dart-gui` is Filament plus ImGui and links core only. An internal
    `dart-gui-imgui` interface gives the backend objects the same ImGui
    headers and definitions. `dart-external-imgui` forwards.
  - The OSG nanobind bindings were removed. Codex ported the tested camelCase
    surface to nanobind as the interim `dartpy.gui` (D31a).
  - `main`'s model-asset demo scene uses the Filament demo types; the Fetch
    and Hubo puppet scenes were retired with `main`.
  - `tools/package/gui.cpp` exercises the Filament viewer. The relocated GUI
    consumer enables C, because the installed ODE finder now calls
    `check_c_source_compiles`.
- M6a changes:
  - C21 removal hygiene, missing-SDK errors that name `DART_BUILD_GUI=OFF`,
    and 7.0 wording.
  - GUI tests that need a graphics device are excluded from
    `tests_and_run` and from the Python `pytest` target; `test-gui` runs them.
  - The capture smoke test is opt-in.
  - The one explicit GUI `gil_scoped_acquire` raises when it cannot attach.
  - The private SDK stages Filament's Vulkan headers.
- Results:
  - `pixi run check-lint` passes, including `check-ai-infra --semantic-cmake`.
  - `test-ai-infra` 1661, `test-build-requirements` 148,
    `test-perf-regression` 870, and `test-gz-compat-tools` 55 pass.
  - `pixi run test-all`: CTest 171/171 and the Python `pytest` target
    697 passed.
  - `pixi run test-gui`: 12/12 `gui` CTest entries; Python GUI and integration
    256 passed. Codex also measured 256 on the GTX 1660 Ti (Vulkan), 256 on
    Lavapipe, and 805 for the full Python suite.
  - Native `FilamentNative.*` 4/4. `offscreen_capture_smoke` passes with
    `DART_ENABLE_GUI_SMOKE_TESTS=ON`.
  - `test-package-core` 2/2. `tools/package` with
    `dart;io;gui;external-imgui;utils;utils-urdf` 4/4. The relocated C++17
    GUI consumer renders a capture.
  - nanobind probes: GUI RSS growth was 4 KB over 1,000 lifecycles; the GC,
    GC-clear, GC-alias and secondary-input probes pass.
  - Configure checks:
    - `DART_BUILD_GUI_OSG=ON` fails with the migration text.
    - `OFF` caches a headless build that survives reconfigure, and
      `DART_BUILD_GUI=ON` re-enables the GUI.
    - Without Filament, configure fails naming `DART_BUILD_GUI=OFF`.
    - Installed `gui-osg`: REQUIRED fails with migration text; OPTIONAL
      leaves `DART_gui-osg_FOUND` false and defines no target.
  - Wheel image: AlmaLinux AppStream Clang 21.1.8 and GLFW 3.4 build the
    pinned Filament SDK in about 154 s. The first end-to-end cp312 abi3 run
    failed because `BlueVK.h` needs Vulkan headers the image lacks, and the
    second failed linking the SDK's non-PIC `libzstd.a` into the extension.
    Staging Filament's Vulkan headers and building the SDK position
    independent fix both. `python3 scripts/wheel_linux.py cp312` now builds
    `dartpy-6.19.5-cp312-abi3-manylinux_2_28_x86_64.whl` (17.3 MB). The
    repair, verification and abi3 audit pass, and so do the smoke tests in
    fresh cp312 and cp313 manylinux containers.
- Post-port reviews: a Codex review of `767f26a0008` (8 findings: 7 real,
  1 refuted because MSVC builds are static) and two lenses of a review
  workflow (docs/AI and downstream; 14 findings). Every confirmed item is
  fixed, with regressions where practical:
  - PickInfo owners join Python GC.
  - Legacy OSG cache migration: a cached `DART_BUILD_GUI_OSG=ON` default is
    dropped, while explicit `ON` still fails.
  - Embedded `DART_BUILD_GUI_OSG=OFF` stays headless.
  - Optional `gui-osg` probes report not found instead of failing.
  - The `gui-device` label keeps device-free GUI tests in the required
    gates.
  - The installed ImGui floor is 1.92.8.
  - Examples detect the Filament GUI by feature rather than by version.
  - The perf harness reports OSG-era arms without OSG as an infrastructure
    error and builds historical OSG arms with both switches.
  - Windows `test-gui` uses the Windows tree; ai-doctor reports Vulkan
    capture readiness.
  - The private SDK is built PIC.
  - Docs follow.

  After the fixes:

  | Gate | Result |
  | --- | --- |
  | `check-lint` | passes |
  | `test-ai-infra` | 1,694 |
  | `test-build-requirements` | 156 |
  | `test-perf-regression` | 875 |
  | `test-gz-compat-tools` | 55 |
  | `test-all` | 178 CTest and 697 Python |
  | `test-gui` | 13 CTest and 257 Python |
  | `pixi run -e gazebo test-gz` | gz-physics 199/199 and 4/4, gz-sim 1/1 |
- Not covered here: in A/B mode, CI perf runs the base revision's harness, so
  this transition PR's A/B against an OSG base cannot build in an environment
  without OpenSceneGraph.

## Completion session (2026-10-10, uncommitted working tree)

Logs and images from this session stayed in the agent session's private
working area; like the `/tmp` packets below, they are not committed.

### Native FTexture use-after-free

- Signature: `utils::PostconditionPanic` in `HandleAllocator.cpp`
  `handleCast:131`, "use-after-free of Handle ... tag=FTexture" on RTX 5070 Ti
  OpenGL. `FTexture` is Filament's default tag for unnamed textures.
- Root cause (Filament 1.77.3 source): `MaterialInstance::setParameterImpl`
  stores only the raw `HwTexture` handle for textures whose handle cannot
  mutate (`Texture.cpp:717`: single level, not external), so `commit()` never
  validates it. ImGui atlas textures are single-level. `imgui_overlay.cpp`
  destroyed a retired atlas texture while the base instance and retained clip
  instances (one per draw command beyond the first, never shrunk) still cached
  it. UBO batching is on by default at feature level 1; shared-UBO growth
  reassigns every instance's buffer, `DescriptorSet::commitSlow` recreates the
  set and resends every cached sampler, and the GL backend
  `GLDescriptorSet::update` handle-casts the dead handle. Vulkan reports the
  same defect as `PreconditionPanic: Handle ... (Texture) is being used after
  it has been freed`.
- Fix: before destroying a retired ImGui texture, rebind the base and every
  retained clip instance to the overlay's lifetime 1x1 fallback texture.
- Red/green: the pre-fix overlay aborts
  `FilamentNative.RetiredImGuiTextureLeavesNoCachedSamplerReferences` with the
  exact signature (`id=27167, tag=FTexture`), and both headless Vulkan
  scenarios (retire on an empty frame; retire on a frame with fewer commands)
  with the precondition panic. The public-API test
  `FilamentNative.ViewerSurvivesAtlasGrowthScaleChangesAndRepeatedWindows`
  (dynamic font sizes, varying window counts, GUI-scale changes, window
  recreation, a second viewer) aborts 3/3 pre-fix with
  `use-after-free of Handle ... tag=FTexture`. All pass after the fix (3/3
  repeats). Two independent read-only analyses (an audit agent and Codex)
  confirmed the causal chain against the Filament source; no other DART
  texture path can produce this panic.

### Other corrections in this session

- `external-imgui` is configured before the GUI-off return again;
  `test_Issue2671` runs in the default gate.
- Backend sources opt out of unity batching (three same-named file-local
  helpers); GUI sources are format-registered in GUI-off builds; format lists
  are de-duplicated.
- `DefaultEventHandler.hpp` includes `ShapeFrame.hpp` (copying `PickInfo`
  failed with only the public header).
- Key input: layout/Shift-aware printable symbols (incl. Windows AltGr and
  keypad digits), held-key repeat, and releases that report their press
  symbol (`detail/key_input.hpp`, unit test
  `FilamentBackend.KeySymbolsFollowTypedCharactersAndRepeat`).
- Viewer: default FOV 30 degrees; DPI scale seeded at window creation; failed
  capture writes no longer repeat; widget exceptions recover without Debug
  ImGui asserts; window input runs only inside its own viewer's frame (queued,
  with button/modifier state captured on arrival, capped at 1024 events);
  frames and graphics setup throw from foreign event callbacks; windows
  released during an event poll are destroyed after it returns.
- Python: no GIL release in `frame/run/setUpView*`; `DefaultEventHandler` is
  not constructible from Python; BodyNode drags skip the shared_ptr owner
  lookup; GUI-off `dartpy.gui` names `DART_BUILD_GUI=OFF`; tutorials use direct
  `dart.gui` inheritance again with weak viewer references; the overlay pixel
  regression uses the 2-unit temporal-dither bound.
- `contact_benchmark` builds without the GUI; the perf harness enables the OSG
  GUI only when the environment provides it; `--cycle-scenes --headless`
  renders every step (`examples_demos_cycle_render`); partial mesh renderables
  are destroyed on exceptions; the installed consumer renders a capture; the
  install check stages only C++ components.
- CI: `ci_filament.yml` adds the demo unit tests, ImGui regressions,
  tutorial/integration tests, a 24-bit Xvfb screen and upload-artifact v7; the
  Release agent smoke runs display-free on Lavapipe; CodeQL builds the GUI
  tree; the nightly unity job enables the GUI; OSG-era GLU packages dropped.

### Results on the final local source

- `pixi run check-lint` passes (C++ format with GUI sources registered,
  CMake, Python, spelling, AI commands, AI infrastructure);
  `pixi run test-ai-infra` 582 passed; `pixi run test-build-requirements`
  121 passed; perf harness and measurement tests 868 passed.
- `pixi run test-gui`: 13/13 display-free `gui` CTest entries (incl.
  `examples_demos_cycle_render` and the demo GUI-scaling and
  memory-diagnostics units) and Python GUI plus integration 187 passed (one
  GUI-off-only skip). Native `FilamentNative.*` 4/4 on RTX 5070 Ti OpenGL;
  ImGui regressions 2516/2668/2671 pass in the GUI tree.
- Rendered 37-scene headless cycle x2 passes; with the ASan allocator
  preloaded (no address reuse) and 12 frames per scene it also passes.
- Python demo catalog `--cycle` 9/9; headless captures of hello_world,
  dynamic_joint_constraints, atlas_puppet, sleeping and the Python atlas_puppet
  were viewed: workspace text, labels, shadows and models render correctly.
- The final native `dart-demos` launch on RTX 5070 Ti OpenGL ran 25 s with
  no panic (stopped by timeout); its captured window was viewed: neutral
  "DART demos" title, docking workspace, gizmo, about 1,225 FPS.
- Display-free `agent-capture` on Lavapipe (llvmpipe, Mesa 26.2.4) passes and
  its image was viewed (contacts, collision bounds, labels).
- GUI-off core gate `pixi run test-all`: 168/168 CTest entries (incl.
  `test_Issue2671`); Python 394 passed, 118 skipped (GUI tests), including the
  GUI-off attribute-error test. `contact_benchmark` builds and runs headless
  without the GUI.
- `pixi run -e gazebo test-gz`: gz-physics 199/199 and 4/4, gz-sim 1/1 (run
  on the near-final source; later edits touched no core or GUI-off build code).
- GUI unity check (whole-target batches, GUI on): 11 GUI/dartpy/demo unity
  translation units compile.
- Relocated installed C++17 consumer compiles, links and renders a
  display-free capture; the live environment's site-packages is untouched.
- CP313 wheel (private source SDK): build, repair (manylinux_2_39), verify and
  smoke test pass; SHA256
  `9cbe9a390b9cde37b51f558511615e1ab647e1c7e9fcea25149e8bcd215391d4`,
  18,172,157 bytes. A clean venv install matches all 21 license/notice files
  and RECORD hashes, passes lazy import and exception translation, and passes
  63/63 GUI cases on the default GPU and on explicit Lavapipe.
- Not run here: native Windows/macOS rendering (needs `CI Filament` after
  approved publication) and accepted OSG performance comparisons (needs a
  quiet host; other sessions' builds kept this host busy).

## Historical qualification

All results below retain their original checkpoint and artifact identities.
They qualify the preview implementation before complete replacement, and do
not close the later reported native texture panic. Historical process-cleanup,
busy-host and outstanding-plan statements describe the time of those records.

### Native default-window correction

Only `Viewer.cpp` and `test_FilamentBackend.cpp` changed after specular-final.
Focused source checkpoint: `/tmp/dart-filament-source-freeze-default-window.json`.
Its 222-file feature map SHA256 is
`34d7b339a81d76b72aff4fb665899b0489ed03f6f595ec1f9c69386127bd117c`;
the 971-file benchmark snapshot SHA256 is
`70b23226abbec2ec8a781268d2fe3606af9ae59f5e809a494a2d9a2836bfb034`.
The benchmark file map changes only `Viewer.cpp`.

GLFW reported primary workarea `[6144,1296,0,1080]`, producing a mapped but
unusable 1x972 window at `(6144,1434)`. The default Viewer now falls back from
an invalid workarea to monitor position and a valid video mode, or 1600x1000.
The corrected native window is 1728x972 at `(6240,1434)`:
`/tmp/dart-filament-native-window-{before,after}/result.json`.
Root viewed the after `window.png`: the UI and RGB gizmo are visible.

Actual RTX 5070 Ti OpenGL passes both native fixtures, including the public
no-argument Viewer dimensions, red-center full RGBA capture and alpha 255.
Root viewed the 1728x972 red-box/blue-background fixture. Evidence:
`/tmp/dart-filament-default-window-native-tests.log` and
`/tmp/dart-filament-default-window-native-fixture/default-native-window.png`.
All 32 GuiViewer cases pass. `HeadlessPixelsAndSceneReplacement` passes its
three cycles on actual GTX 1660 Ti Vulkan, not the RTX device. Logs:
`/tmp/dart-filament-default-window-{viewer,headless}-tests.log`.

The then-current preview launch task (since replaced by `pixi run demos`) configures, completes its build and
opens the mapped 1728x972 window at `(6240,1434)`. Root viewed its PNG:
`/tmp/dart-filament-default-window-pixi-launch.{json,png}`.
The terminal was intentionally interrupted after capture; this is not a runtime
failure. All native, headless and demo processes are drained. `pixi run build`
and three GUI targets pass; the normal build log is
`/tmp/dart-filament-default-window-core-build.log`. Both code files pass the
clang-format dry run, and two bounded production reviews are clean. Full lint
passes at `/tmp/dart-filament-default-window-lint.log` with both source maps
unchanged.

The following full source, catalog, wheel, functional, resource and acceptance
packets qualify the previous specular-final bytes. The startup checks above do
not renew that full qualification for the changed source.

### Previous specular-final checkpoint

Full lint, optional C++ formatting, check-lint and 13 GUI targets pass. Both
actual RTX and Lavapipe pass 114 focused C++ cases (18/9/32/10/6/8/31) and
37-scene cycles; native Linux passes one case. CP314 passes 65 cases per device,
zero skips. Shared and SOURCE static relocated C++17 consumers pass. Logs use
specular-final.
Both reviewers match all 222 feature files, SHA256
`5a815f65ca5f059b22155244795f7ae72d7b7c97cbf49ba7753b89050c8653ef`, and
the 971-file benchmark source snapshot, SHA256
`8632c38ad878fe814e7072ff1e88c06c1d1478253be853cab37c738a125d2420`.
Full map: `/tmp/dart-filament-source-freeze-specular-final.json`.

Fetch diagnostics isolate the visible gap to imported all-white legacy specular:
retained OSG rejects all four RGBA components >= 1, whereas the neutral importer
had missed that rule. Assimp STL defaults were confirmed by an actual Fetch
material dump.
Filament then removes diffuse fill, and direct shadows expose black surfaces.
Same-state four-arm captures have 1,038 baseline pure-black pixels versus one with
shadows off. The three-file source/fixture correction restores the exact OSG
rule only at the existing legacy selection boundary; valid RGB/alpha and explicit
PBR factors/maps stay intact. New descriptor and in-memory Assimp mesh shadow tests
retain every old bound and require full RGBA agreement with the black-specular
reference, visible shadowed fill and real ON/OFF shadow contrast. Root viewed those PNGs;
the dark center shadow is readable and disappears when disabled. No lighting,
shader, shadow-feature or quality setting changed. Two benchmark sources were
also formatted; whole previous contents are unavailable, so reviewers verified
current zero-MSAA guard correctness rather than full-file whitespace equivalence.

Rebuilt repaired SOURCE CP313 wheel passes build/repair/verify/test, exact 21
notices/metadata/RECORD, ABI/stack, isolated installed import/exception and static
consumer. SHA256 `ff16a6c592fcd48952193029acc9fb27a7772cd544d723122944349aa112199b`,
23,141,294 bytes, manylinux_2_39; ABI floors unchanged GLIBC 2.27,
GLIBCXX 3.4.31/CXXABI 1.3.15, no LLVM runtime, GNU_STACK RW. Evidence:
`/tmp/dart-filament-wheel-specular-final`. Installed GUI checks pass 63 cases on
RTX and 63 on Lavapipe, zero skips. Both red/blue capture pairs change 8,435
pixels; full 307,200-byte RGBA images, hashes, alpha 255 and device/ICD/LP16
metadata are preserved. All source maps match afterward and the wheel owner
has drained every process.

Fresh public displayed DEPTH passes all 12 arms, original bounds and actual
device guards at `/tmp/dart-filament-default-depth-qualification-specular-final`.
The 12 opaque textured point/line captures pass exact full RGBA invariance across
texture alpha 0/128/255. Freshly assessed supplements also retain uniform x/y
translucent faces and fixed-direction ground shadows with headlights on/off.
Imported white-specular and black-specular references differ by at most two
RGBA units; the shadowed center remains readable, and disabling shadows lifts
its red channel by 93. Assessment:
`/tmp/dart-filament-visual/qualified-supplement-specular-final/semantic-assessment.json`.

Fresh smoke captures pass all nine pairs (18 PNGs); the canonical catalog passes
all 37 pairs (74 PNGs), including bounded semantic review. Every paired capture
has exactly 30 finite steps and matching model/physics and completed-capture
state hashes. Actual device, PNG integrity, full source, executable and all
resolved-library guards pass before/after: 60 OSG and 56 Filament libraries.
The image reviewer opened all seven catalog sheets, all nine smoke original
comparisons, and original Fetch, Atlas, Hubo, operational-space arm and soft-body
comparisons. Assessment packet:
`/tmp/dart-filament-visual/qualified-catalog-specular-final/assessment-packet-index.json`.

The former Fetch P2 is closed for this real canonical view. Restored under-head,
arm-joint, torso and base surfaces remain readable, with meaningful shadows.
The same raw-image ROI `(545,270)-(790,520)` now has zero pure-black RGB pixels
and zero pixels with maximum RGB <= 5, versus 1,038 and 3,352 at depth-final.
Both renderers and checkpoints match completed-capture state hash
`66e67360bfaa5921`. Resolution and image hashes:
`/tmp/dart-filament-visual/qualified-catalog-specular-final/fetch-appearance-resolution.json`
and its `.md` companion.

These are bounded canonical-view passes. Neutral-material brightness, shadow
softness, caption glyph metrics and extra hardcoded-design wire diagonals still
differ; some canonical views clip subjects or show them too small to assess
fully in both renderers. The optional ssik module is unavailable, with zero
skeletons in both sampled states. Static captures do not prove live input, DPI,
all alternate cameras/modes or later dynamics. All owned capture and integrity
processes are drained.

All 16 fresh functional benchmark arms pass across four scenes, two renderers
and two devices, including completed frames, full RGBA and paused-state guards.
The functional manifest records unchanged source, executable, helper and core
library identities; it does not record the full resolved-library maps used by
the catalog. Evidence:
`/tmp/dart-filament-benchmark-functional/all-specular-final-functional-manifest.json`.
These runs establish function. Accepted timings, resource acceptance, final
acceptance reviews and native Windows/macOS CI remain separate release gates.

### Previous depth-version-final binaries and runtime

Depth-version-final full lint, optional GUI formatting, check-lint and all 13
GUI targets pass. Both display-free RTX and explicit Lavapipe pass all 112 C++
cases (17 descriptor, 9 interaction, 32 Viewer, 10 parity, 6 decoder, 8 mesh,
30 backend) plus the 37-scene activation cycle. Native Linux passes its case;
CP314 passes all 65 focused GUI/import cases on each device, zero skips.
Logs: /tmp/dart-filament-cpp-{gpu,cpu,native}-depth-version-final.log and
/tmp/dart-filament-python-{gpu,cpu}-depth-version-final.log.

Both independent source reviews match the complete 222-file feature manifest
SHA256 dc0bb16f9be0caf50f2937fdb9d44bef2ea761eeb2630b50c4e441a9d72dd25a
and 971-file benchmark snapshot SHA256
82590c973184b6e6f4eef0a210ef5e2e53acaae67252e2846fe34392c094ff5d.
Full map: /tmp/dart-filament-source-freeze-depth-version-final.json.
The initial GPU depth-final run passed 111/112; one new fixture edited geometry
under a retained ID without advancing its resource version. Four monotonic
version updates correct that fixture; every numeric and image bound remains.
This is the only delta from the previous depth-final freeze. All production
sources, shared libraries, wheel bytes and benchmark snapshot are unchanged.

DEPTH now preserves original per-primitive culling and depth-write policy using
three shared override instances, fits visible successful geometry to OSG's
primitive range (.98/1.02 margins and .0005 ratio), uses a finite custom
projection, clears far-white and restores authored COLOR camera/background.
Actual public Viewer captures on RTX Vulkan and native NVIDIA OpenGL match
OSG single gray130, two surfaces14/246 and crossing-floor7..253; a floor sample
is88 versus87. COLOR round trips differ by at most2. Fixed-clip OSG controls
reproduce old Filament254/255, establishing the original contrast defect.
All12 arm/state/full-RGBA guards pass in
/tmp/dart-filament-default-depth-qualification-depth-final/records.json.

Factory policies now match original ordinary/mesh alpha thresholds, every
material alpha channel and MeshAlphaMode, PointCloud BOX blend/write exceptions
and vertex-color nonwriting policy. Opaque mesh points/lines previously vanished
with texture alpha0 while OSG retained625point/3258line pixels; actual diagnosis
is preserved in /tmp/dart-filament-textured-proxy-alpha-diagnosis. One existing
point_color shader now compiles into transparent and opaque packages; the
opaque branch forces alpha1 after sampling without RGB premultiplication.
The full GPU/CPU suite passes opaque full-RGBA alpha0/128/255 invariance and
retained transparent/vertex-color fade controls. Fresh public12-capture qualification also passes on actual RTX and native
OSG: all three texture alphas produce identical full RGBA (maximum delta0),
with625 point pixels for each renderer and3250/3258 line pixels. The assessed
composite, metrics and immutable provenance are preserved in
/tmp/dart-filament-textured-proxy-alpha-qualification-depth-final. Full source
and library fingerprints match before/after and every process is drained.
Native platform and performance acceptance remain separate.

Depth-final shared/static relocated C++17 consumers, cached SOURCE wheel
build/repair/verify/test, 21 exact notices/metadata/RECORD, isolated installed
lazy import/exception smoke and ABI/stack pass. Repaired wheel SHA256
3cae7a180aba392b16e096264a1f850680e52b05a1846e0a2573da12d668a086,
23,140,370 bytes, manylinux_2_39. Module floors remain GLIBC2.27,
GLIBCXX3.4.31/CXXABI1.3.15, no LLVM runtime, GNU_STACK RW. Evidence:
/tmp/dart-filament-wheel-depth-final and
/tmp/dart-filament-shared-consumer-depth-final.log. Installed graphics pass63
cases on each actual driver, zero skips; both changing-color captures change
8435pixels. The wheel owner has drained all processes. Evidence logs:
/tmp/dart-filament-wheel-installed-{gpu,cpu}-depth-final.log and
/tmp/dart-filament-wheel-installed-{rtx,lavapipe}-pixels-depth-final.log.

Fresh9smoke and37catalog paired captures pass state/source/library/device and
92PNG integrity guards. Semantic assessment clears36canonical views and
retains Fetch as an unresolved appearance gap: ROI545,270–790,520 has
1038pureblack Filament pixels versus OSG0 and previous coefficient Filament1.
Root viewed the comparison; black patches under the head, arm joints and
torso/base hide gray material detail. This earlier finding is closed by the
specular-final real-asset assessment above.
Evidence: /tmp/dart-filament-visual/qualified-{smoke,catalog}-depth-final.
No accepted timing or current600s resource pass exists.

All16 functional benchmark arms pass across four scenes/two renderers/two
devices, including completed frames, fullRGBA and paused-state guards; source,
core library and helper identities match before/after. The actual OSG zero-sample
guard executes before warmup. Evidence:
/tmp/dart-filament-benchmark-functional/all-depth-final-functional-manifest.json
and {gpu,cpu}-depth-final. These short runs establish function, not performance.

### Historical checkpoints

Transparency-final corrects specialized Box Y-face winding and applies OSG
color culling at the shared descriptor boundary, preserving SoftMesh two-sided
surfaces and existing shadow culling. Full lint, GUI formatting, check-lint,
GUI build, shared/static C++17 consumers, wheel ABI/import/exception smoke and
21 notices/RECORD pass. Wheel SHA256:
`5dd1d121dc6752afeeefcc9486b83681dcf3a55a067cac508efe6de625d4bd88`.
GPU passes 107/109 cases; CPU passes 108/109. Only the two new fixtures fail
unproved all-channel/literal-background assumptions; all preceding 107 cases
pass. The measured red-band oracle and independent blank-background capture
will replace those assumptions without changing production or old thresholds.
The corrected translucent-face PNGs are visibly uniform. Both independent
reviewers found a separate P2: DEPTH binds one two-sided material and bypasses
the color culling policy. Backend owns its bounded reproduction and correction;
catalog, installed-wheel graphics, timings and soaks hold. Logs:
`/tmp/dart-filament-cpp-{gpu,cpu}-transparency-final.log`.
Preserved source map:
`/tmp/dart-filament-source-freeze-transparency-final.json`, feature SHA256
`43b1c214ac7f72ff2439766434847e830e95322ee9cae0425ae7d6150e7eb799`,
benchmark SHA256
`298411b0a4884c4475eb4b5f86547508fce39b449fbace3cf4fe47c451c0e12a`.

Height-light-final corrects cardinal heightmap normals and exclusive headlight
mode, calibrates the sole camera light to 70k and preserves fixed-light1
shadow direction. Full lint, GUI formatting, check-lint and GUI/wheel builds
pass. Both RTX and Lavapipe pass 106/107 C++ cases and all 37 activation scenes.
Only AlphaFadesLitAndUnlitGeometry fails: opaque red 87 versus half-alpha 83
does not satisfy the retained 20-unit increase. Backend owns actual OSG and
shadowed/unshadowed diagnosis before any source or fixture change. Logs:
`/tmp/dart-filament-cpp-{gpu,cpu}-height-light-final.log`.
Actual OSG alpha0/.5/1 red is 0/127/255 with default shadows off and 0/127/127
with explicit fixed-direction self-shadows. The same assertion therefore
fails under OSG shadows. The alpha-only fixture now explicitly disables
shadows, preserving all bounds. A separate visible translucent self-shadow
banding artifact was traced to inward specialized Box Y faces and rendering
hidden back faces. The bounded candidate corrects that winding and applies
OSG's color-pass BACK culling to descriptor-created surfaces, except SoftMesh;
each existing shadow-culling mode is preserved. Actual RTX Vulkan, NVIDIA
OpenGL and Lavapipe probes show uniform two-angle alpha faces within two
display levels, unchanged opaque images within two levels and retained ground
shadows. Production edits and two regressions await fresh qualification.
Original Filament half-alpha red varies 78..104 versus flat OSG 127. Evidence:
`/tmp/dart-filament-alpha-shadow-diagnosis/box-alpha-shadow-comparison.png`.
Candidate evidence:
`/tmp/dart-filament-alpha-shadow-diagnosis/qualified-winding-summary.json`.
CP314 CPU passes all 65 focused GUI/import cases, zero skips:
`/tmp/dart-filament-python-cpu-height-light-final.log`.
Prior coefficient-final passes below do not qualify these new binaries.

Fresh relocated shared and static C++17 consumers, repaired wheel ABI,
isolated import/exception smoke and all 21 license bytes/RECORD pass. Wheel
SHA256 `696e63424e9f9ab5e0f5ff5727d348d1c972eba3bfef642ce0c7149e011e92cb`;
preserved `/tmp/dart-filament-wheel-height-light-final`.
Source freeze: 222-file manifest
`666aed4a6ac0124709813e86c30120cce5d82be231f4f04061e0684cb73573f8`;
benchmark snapshot
`46e23e6f79f89be1998282e37051a643d425a3dee463cc2b3ce2216f68819917`.
Full map `/tmp/dart-filament-source-freeze-height-light-final.json`.

The retained OSG headlight oracle is captured on RTX at camera positions
`(0,-4,0)` and `(4,0,0)`: authored gray .25/.5/.75 gives RGB 64/127/191
with headlights on at either angle; fixed-light mode gives 26/53/79 and
13/25/38 respectively. Evidence:
`/tmp/dart-filament-headlight-calibration/osg-corrected-gpu/gray-samples.json`.
Earlier gray samples reused static OSG color caches and are invalid. Filament
calibration uses fresh geometry, exclusive lights and unchanged tone/exposure.

- Full optional-GUI build passes for both libraries, both C++ demo hosts,
  all GUI tests, benchmark and CP314 Python bindings:
  `/tmp/dart-filament-final-gui-build-coefficient-final.log`.
- Pre-color/capture checkpoint: all 100 C++ cases pass on both display-free
  RTX and Lavapipe, plus native Linux capture. Initial PBR emissive alpha and
  point/line fixture camera synchronization defects were resolved without
  changing pixel thresholds. Logs:
  `/tmp/dart-filament-cpp-{gpu,cpu,native}-emissive-final.log`.
- Latest source corrects authored-color transfer at the shared input boundary.
  A controlled default-blue wire comparison gives median RGB OSG 96/96/192,
  old Filament 148/148/205, standard sRGB-converted Filament 94/95/194.
  `/tmp/dart-filament-wire-color-proof/color-transfer-proof.json`.
- Latest source queues RGBA within one frame and consumes it exactly once for
  PNG, recording and raw capture. New cases cover changing uploads, single
  refresh, empty/populated/hidden overlays, recording and resize. Fresh GUI
  runtime qualification now passes. The GUI build caught a new fixture
  callback typo; corrected customPreRefresh genuinely compiles and the RTX
  combined-capture regression passes:
  `/tmp/dart-filament-combined-capture-proof.log`.
- Independent source review found linear dark RGB loses precision when packed
  into UBYTE4 (authored 0.025 becomes exact zero). Backend owns the minimal
  HALF4 fix and actual rendered regression now pass. Two new point fixtures use
  pixel Point mode instead of world-sized BillboardSquare. Production tone
  mapping is unchanged and can still map near-black gray to zero; the precision
  fixture isolates storage with LinearToneMapper, not production dark-color
  parity.
- Clamped HALF4 checkpoint builds successfully and passes 101/105 C++ cases
  on both RTX and Lavapipe. Four backend cases fail on both devices: queued
  capture pure-channel assumptions, no-UV blue JPEG contrast, legacy specular
  and emission, normal-map contrast. Native Linux passes; the all-scene
  activation cycle passes on both devices. Backend diagnosis is open; preserve
  thresholds. Logs `/tmp/dart-filament-cpp-{gpu,cpu,native}-clamp-final.log`.
- The four failures are resolved at coefficient-final without reducing any
  material contrast threshold. Ambient/specular F0/emissive coefficients remain
  raw linear: actual OSG emission blue 51, corrected Filament 49, prior converted
  Filament 2. Diffuse fixtures encode their intended old linear shader inputs.
  Queued capture uses independent same-state RGBA within two display units and
  keeps exact background/alpha, upload/resize and single-consume checks. Probes:
  `/tmp/dart-filament-four-fixture-diagnosis/checkpoint.json`.
- Fresh coefficient-final C++: all 105 cases pass on both RTX and Lavapipe,
  plus the native Linux case and both 37-scene activation cycles. Logs:
  `/tmp/dart-filament-cpp-{gpu,cpu,native}-coefficient-final.log`.
- Fresh CP314 focused GUI/import checks: 65 pass on each device, zero skips;
  `/tmp/dart-filament-python-{gpu,cpu}-coefficient-final.log`.
- Final coefficient wheel repair, import/exception smoke, ABI, all 21 notices
  and RECORD, and relocated static C++17 consumer pass. SHA256
  `3650c4719c345dd51d58f725e1e90720f80e90925bb1aa9173be3757bcf7814d`;
  preserved `/tmp/dart-filament-wheel-coefficient-final`.
- Final clean installed CP313 wheel: all 63 GUI cases pass on both RTX and
  explicit Lavapipe, zero skips. Both actual changing-color captures show
  8,435 red pixels, 8,435 blue pixels and 8,435 changed pixels. Logs:
  `/tmp/dart-filament-wheel-installed-{gpu,cpu}-coefficient-final.log` and
  `/tmp/dart-filament-wheel-installed-{rtx,lavapipe}-pixels-coefficient-final.log`.
- Full CP314 Python RTX run: 514 passed, zero skips/failures/crashes;
  `/tmp/dart-filament-python-full-rtx-identity-final.log`. Explicit device is
  RTX 5070 Ti, vendor 10de/device 2c05.
- Display-free Lavapipe Python: 67 passed, zero skips/failures/crashes, 22
  intentional OSG/general deselections;
  `/tmp/dart-filament-python-cpu-identity-final.log`.
- Clean installed repaired CP313 wheel: all 63 GUI cases pass, zero skips,
  on both RTX and explicit Lavapipe. Bare import and invalid-scale exception
  translation pass with library/display/conda paths scrubbed. All 12 identity
  regressions pass without calling subclass hash/equality methods.
  `/tmp/dart-filament-wheel-installed-qualified.log`.
- Installed-wheel CPU capture visibly updates red to blue: 8,435 red pixels,
  8,435 blue pixels and 8,435 changed pixels, actual llvmpipe LLVM 23.1.2,
  Mesa 26.2.4;
  `/tmp/dart-filament-wheel-installed-lavapipe-pixels-qualified.log`.
- Same DemoHost captures twice with one theme hook per context and clean
  destruction. The prior executable crashed in the old context's theme-hook
  destructor. Fresh proof:
  `/tmp/dart-filament-demo-reuse-proof/qualified-runtime.log`.

At that preceding color/capture checkpoint, source and installed artifacts were
qualified on Linux while paired catalog assessment, measured performance/resources
and native CI remained separate gates. glTF diffuse/vertex linear-factor provenance differs from the
retained display-color boundary; this is recorded as a nominal-standard
observation without a reproduced OSG appearance defect.

### Packaging and build closure

- Filament 1.77.3 is exactly pinned with matching material compiler. Twelve
  materials compile for OpenGL/Vulkan/Metal, including four legacy gloss
  variants. Public headers remain C++17; private source-SDK/backend compilation
  uses the SDK's required C++20.
- Production Linux uses the exact source SDK, Clang 22/libstdc++ and sysroot
  2.28. Cold SDK build/staging takes 195 seconds; cached helper 1.3 seconds.
  The upstream Linux binary SDK is excluded after real glibc 2.38 and mixed
  libc++/libstdc++ exception-translation failures.
- Actual CP313 production wheel build and automatic repair pass. Repaired
  artifact is 23 MiB, `cp313-cp313-manylinux_2_39_x86_64`, SHA256
  `27efc7b9f3ef2d90c89cad2c32de679cf97baa2171955b6209bf034b86ca3f7d`
  at the preceding checkpoint. Color/capture repair and relocated static C++17
  consumer pass, SHA256
  `803e4c51da1cf193f3ae6d35e598e5b0a880416e40fd30977d37f6ffddfcfbc6`;
  artifacts preserved in `/tmp/dart-filament-wheel-color-capture-final`.
  Final coefficient wheel rebuild and repair are recorded above.
  `/tmp/dart-filament-py313-wheel-build-emissive-final.log`.
- Final extension requirements: GLIBC 2.27 / GLIBCXX 3.4.31 / CXXABI 1.3.15,
  no libc++/libc++abi, RW stack. Six baseline core objects under the same
  origin/main compiler/runtime lock already impose CXXABI 1.3.15; the unchanged
  GNU policy floor is manylinux 2.39. Intermediate GLIBCXX 3.4.32 references
  disappeared under LTO. A 2.28 sysroot does not establish a manylinux 2.28 tag.
  `/tmp/dart-filament-wheel-final-abi-qualified.log` and
  `/tmp/dart-filament-wheel-origin-toolchain-lock.log`.
- Native installation and both wheel providers use one verified license
  manifest: root Apache license plus 20 upstream runtime notices. Exact LF/CRLF
  root bytes are preserved after validation; malformed or unrelated root
  licenses are rejected. Clean repaired-wheel notice bytes and RECORD hashes
  pass. SOURCE and SYSTEM native staging proofs pass all 21 hashes.
- Fresh relocated source-SDK static installed consumer genuinely compiles,
  links and runs as C++17, including aggregate neutral headers. All 21 notice
  hashes pass. The root pkg-config install destination now honors `--prefix`:
  `/tmp/dart-filament-source-static-consumer-qualified.log`.
- Fresh coefficient-final relocated shared C++17 consumer passes compile/link,
  runtime, private-header exclusion and exact license hashes:
  `/tmp/dart-filament-shared-consumer-coefficient-final.log`.
- Read-only build/provider/wheel requirement and license helper aggregate:
  121 passed; `/tmp/dart-filament-final-build-requirements.log`.

### Core and earlier renderer checks

- Default build and all 168 CTest entries pass (109.89 seconds, parallel 16):
  `/tmp/dart-filament-default-tests.log`.
- Renderer-free Python: 373 passed, 73 configured optional skips. Isolated
  import and simulation link no Filament/OSG/GLFW/ImGui/Vulkan/OpenGL libraries.
- Gazebo patched forward lane's four performance/symbol checks and gz-sim
  entity_system pass. This is not unpatched last-release ABI proof.
- Sphinx HTML builds with warnings treated as errors.
- The preceding renderer checkpoint passed 90 focused cases on GPU and CPU,
  plus native RTX OpenGL resizing/readback/orientation/three-context teardown.
  Source fixes preserve premultiplied alpha, authored one-pixel
  line colors, outward capsule faces, normal-map handedness and cascaded ground
  shadows. New cases must pass before reusing these as final qualification.
- Bounded repeat-only teardown passes for 102 RTX, 102 Lavapipe and 30 native
  contexts. These are functional repetitions, not the required ten-minute soak.
- Prior paired captures of all 37 demos have equal finite 30-step model/physics
  and final capture-state hashes. Geometry, poses and framing align, but real
  background, label layering and WAM no-UV texture defects were corrected and
  assessed in the emissive-final catalog. Those images exposed pastel authored
  colors; latest color/capture binaries require fresh assessed pairs. Prepared
  helpers and assessments are under `/tmp/dart-filament-visual/`.
- The actual prior native and headless main-view configuration disabled
  MSAA/FXAA. Unused quality branches are removed; there is no native 4x MSAA
  qualification or change in active quality settings.

## Measurement boundaries

Completed functional/resource packets below qualify the previous specular-final
bytes; their immutable source and artifact identities remain unchanged.

The common driver measures paused-world facade frames with actual completion:
OSG uses a glFinish final-draw callback without buffer swap; Filament capture
frames already flush and wait without a redundant second completion. Both
cameras use 45-degree FOV and fixed 0.01/1000 clipping. Flushed phase markers
exclude warmup. `readback_full_frame_ms` includes one facade frame on each side
and queues readback in that same frame; PNG encoding is separate. Scene-color
coverage rejects a floor without objects, and exact paused-state hashes must
match. OSG node ownership and duplicate default soft meshes were corrected.
This compares supported default pipelines with matching DART behaviors, not
identical internal shader operations: Filament retains PBR-neutral color
grading, HIGH HDR and dithering; OSG renders directly to RGBA8. Both disable
FXAA and default to camera headlights with shadows off. Filament disables
MSAA; the actual OSG zero-sample guard passes in all eight fresh OSG arms. The public
Filament WorldNode overrides the private backend's initial shadow flag before
the measured frame. Optional shadows use four Filament cascades versus OSG's
single map; the first accepted boxes comparison has shadows off. Preserve
production quality instead of reducing it to pass timing/resource gates.
All eight arms per device pass at the current checkpoint under
`/tmp/dart-filament-benchmark-functional/{gpu,cpu}-specular-final`; their manifest
records core library identities, not full resolved-library maps. Rerun after
any production change.

The coefficient-final ten-minute GPU observation completed 845,889 frames in
600.000375 seconds. RSS settled at 285,440 KiB; all 583 measured-phase GPU
samples were 70 MiB. The run failed its predeclared minimum 595 GPU samples
and is not an accepted resource pass. Independent raw-log diagnosis finds
no missing PID rows: continuous `pmon -d1` actually averaged 1.027267 seconds
per cycle, with sixteen recurring two-second displayed timestamp gaps. RSS
sampling independently produced 600 rows. The corrected absolute-deadline
NVML sampler passes 16 fake checks and 20 seconds of live sampling: 20 NVML rows,
no errors or missed deadlines, .999993s cadence, maximum query 7.06ms and
lateness 5.77ms. Raw 70.0390625 MiB agrees with pmon 70 within its integer display
precision. Coverage guards remain 595 GPU and 597 RSS samples. Evidence:
`/tmp/dart-filament-resource-soak-sampling-live-qualification-precision/probe.json`
and
`/tmp/dart-filament-resource-soak/gpu-coefficient-final/assessment.json`.

The specular-final GPU observation completed 671,944 point/line frames in
600.000520 seconds, with 600 whole-phase RSS and GPU samples, zero missed slots
or query errors. Full 1280x720 RGBA, state, device, binary and cleanup guards pass.
Own-process GPU memory remains 73,441,280 bytes (70.0390625 MiB); first/last
60-second RSS medians are both 285,868 KiB, after an initial 278,188 KiB.
Independent audit passes all source, library, device and cleanup guards. The
late 300 seconds remain exactly 285,868 KiB RSS and 73,441,280 bytes GPU memory.
Assessment:
`/tmp/dart-filament-resource-soak/gpu-specular-final-independent-assessment.json`
and its `.md` companion. Packet:
`/tmp/dart-filament-resource-soak/gpu-specular-final`.

The first CPU attempt exited with shell status 143 before `measured_end` or a
final record. Its wrapper and benchmark are gone; no summary or renderer-crash
diagnostic is present. The exact sender and cause of the indicated external
termination are not established. Preserve the incomplete packet at
`/tmp/dart-filament-resource-soak/cpu-specular-final`; it is rejected as resource
qualification:
`/tmp/dart-filament-resource-soak/cpu-specular-final-interrupted-assessment.json`.
A fresh CPU retry completed with exit 0 in 600.006953806 seconds, with 31,728
completed point/line frames and 600 whole-phase RSS samples. No slots were missed
and no query errors occurred. Full RGBA, state, device, binary and cleanup guards
pass, and the benchmark PID is gone. Before/after full 222-file feature and
971-file benchmark maps, binary, 50 resolved libraries, helper and ICD identities
match. Packet: `/tmp/dart-filament-resource-soak/cpu-specular-retry`; full maps:
`/tmp/dart-filament-resource-soak/cpu-specular-retry-before.json` and
`/tmp/dart-filament-resource-soak/cpu-specular-retry-after.json`.

CPU RSS oscillates: first/last 60-second medians are 242,424/251,712 KiB, the
range is 235,648..251,748 KiB, and 222 sample steps are negative. RSS already
reaches 251,744 KiB during elapsed 60–120 seconds; later ceilings are at most
251,748 KiB. The whole-run fit is 394.77524 KiB/min and the fit for the last
300 seconds is 314.97499 KiB/min.
These slopes alone do not establish an ongoing leak; no arbitrary threshold,
leak-freedom or no-regression claim is made. Independent CPU audit passes the
bounded source, library, helper, ICD, state, device, PNG and cleanup guards.
Assessment:
`/tmp/dart-filament-resource-soak/cpu-specular-retry-independent-assessment.json`
and its `.md` companion.
Neither observation establishes comparative timing or release resource
acceptance: unrelated builds including task_10 were active, and accepted
comparisons remain pending. Fresh preflight still shows task_6 Release/Gazebo
and scratch performance builders. The maintainer plans a 30-minute quiet window;
root has not stopped others' work or launched accepted timings on this busy host.

The runner requires explicit hardware, one thread per physical core, empty
output and unchanged source/binary/resolved-library/compiler/build/lock
fingerprints. GPU steady-memory intervals must lie wholly within the measured
phase; peak is the maximum observed one-second own-process sample, not an exact
instantaneous driver high-water mark. Twenty runner tests and ten wheel/runtime
helper tests pass. No accepted renderer performance samples exist.

Timings require an idle host. Unrelated builders and integration suites can
resume; GUI/wheel builders and qualification must drain first. Recheck host load;
never stop others' work or accept contaminated measurements. Early functional
probes show substantial startup/RSS costs and CPU regressions. These are not
accepted timing samples and do not justify weakening the contract.

Actual RTX device is 5070 Ti, driver 580.178.04, selected display-free by
`MESA_VK_DEVICE_SELECT='10de:2c05!'`. CPU Vulkan uses the explicit conda Lavapipe
ICD (LLVM 23.1.2). CPU OSG fails Mesa pbuffer creation on NVIDIA `DISPLAY=:1`;
glxinfo alone did not prove a working baseline. Actual CPU OSG uses isolated
extracted Xvfb, llvmpipe LLVM 20.1.2. Package 2:21.1.12-1ubuntu1.8, SHA256
`712429d9342c7d6cef63afb33ccfcd2bd477a69371b5c9941a9c6dde31a4cd3c`;
helper owns the server and terminates it, with no system installation.

## Review and pending acceptance

Two earlier independent full source reviews are clean at the coefficient-final
freeze.
They checked public compatibility, callback/observer teardown, retained owners,
identity caches, material dependencies, license provenance and provider/CI
paths. Three new lifetime tests reproduced SIGSEGV with the prior library and
pass with current binaries. Both reviews independently match the 222-file
feature manifest excluding five task ledgers:
`3f807574c1d1294c39ace5ab482cee41a7b6b60fd1cbcac1f75a59b804a9c157`;
benchmark source snapshot:
`2450e5dd15597e79394ed4bb4421fcbd9a5429b7fc61a8f14d6fff4f923b9dba`.
The prior independent source reviews match the specular-final freeze recorded
above. Two final bounded evidence reviews pass with no unresolved findings:
`/tmp/dart-filament-acceptance-review-specular-final-{package,parity}.{json,md}`.
The parity review reconciles all 358 referenced hashes. Both preserve actual
capture formats, resource observation limits and the pending release gates;
these reviews do not establish readiness.

Full lint, explicit optional-GUI formatting and read-only check-lint pass after
the coefficient-final corrections:
`/tmp/dart-filament-{full-lint,optional-gui-format,check-lint}-coefficient-final.log`.
The prior root full lint passes with all 222 feature hashes unchanged:
`/tmp/dart-filament-full-lint-final-review-reconciliation.log`. It precedes this
native-window source changes; current focused lint is recorded above.

Remaining gates: Linux seven-pair/30-second empirical-noise comparisons for
completed frames/setup/full RGBA/RSS/GPU resources; actual native Windows/macOS
build/render/package CI and final base integration. Static CI/package
review found no concrete new defect; these native workflows remain unpublished
and unrun.
Hosted CI requires approved publication. Windows/macOS performance is
maintainer-deferred. No default switch or equal/better claim is established.
