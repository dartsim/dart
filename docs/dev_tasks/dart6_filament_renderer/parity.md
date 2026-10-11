# Parity acceptance matrix

## Current sole-Filament acceptance

The user approved complete OSG removal for DART 7, superseding the staged
6.22-removal plan. C++ core/gz compatibility and nanobind remain required;
Python GUI API changes are approved. The completion session (see
`verification.md`, top section) repaired the native FTexture use-after-free
with deterministic red/green regressions and locally qualified the final
source on Linux: GUI tree C++/native/Python tests, the rendered 37-scene
cycle, Python demos, display-free capture on RTX and Lavapipe, GUI-off core
gates, Gazebo, the CP313 wheel, and the relocated installed consumer.

M6a merged in [#3696](https://github.com/dartsim/dart/pull/3696) on 2026-10-11;
native functional CI passed on Linux, Windows and macOS. Still pending for
release acceptance: the wheel/cache gates in `README.md`, accepted Linux
performance/resource comparisons against an immutable OSG checkout on a quiet
host, and final release reviews. Historical paired visual assessments below cover the
preview bytes; the completion session re-checked rendering with viewed
captures but did not repeat the 37-pair OSG comparison.

## Historical preview qualification

Every result below belongs to its stated old source freeze. The old
native-window and specular-final passes do not qualify the current complete
replacement or close the subsequently reported texture lifetime defect.

The baseline is DART OSG at origin/main `9ccbb700e0d`. Descriptor/input
contracts, rendered checks, paired visual assessments and release acceptance
are separate evidence. Native OSG graph/shader/embedding hooks have migration
guidance in the published visualization guide.

The historical native-window correction changes only `Viewer.cpp` and its backend
fixture after specular-final. Default dimensions recover from 1x972 to 1728x972;
the then-current preview launch task maps the usable window, and root viewed
its PNG. Both actual RTX OpenGL native fixtures and all 32 GuiViewer cases pass;
the three-cycle headless replacement fixture passes on actual GTX 1660 Ti Vulkan.
Normal/GUI builds, two bounded production reviews and full lint pass.
Lint: `/tmp/dart-filament-default-window-lint.log`; both source maps are unchanged.
Focused checkpoint: `/tmp/dart-filament-source-freeze-default-window.json`.
Root's final lint log will be `/tmp/dart-filament-default-window-lint.log`.
Evidence: `/tmp/dart-filament-default-window-pixi-launch.{json,png}` and
`/tmp/dart-filament-default-window-{native,viewer,headless}-tests.log`.

All full source, catalog, wheel, functional, resource and acceptance evidence
below qualifies the previous specular-final bytes. It does not qualify the
latest native-window change; only the focused checks above cover that change.

The previous specular-final checkpoint passes all 114 focused C++ cases on RTX
and Lavapipe,
each 37-scene cycle, native Linux and 65 CP314 cases per device, zero skips.
Both independent reviewers match 222 feature files and the 971-file benchmark
source snapshot at the freeze recorded in verification.md. The import fix restores OSG's exact legacy
specular rejection when all four RGBA components are >= 1, preserving valid
coefficients and explicit PBR. Descriptor and rendered shadow regressions pass,
including visible shadowed fill and real on/off contrast. Fresh public DEPTH
and opaque textured point/line alpha proofs each pass 12 arms with actual
device, full RGBA, state, source and library guards; alpha invariance is exact.
Shared/static C++17 consumers and the rebuilt SOURCE wheel's ABI, 21 notices,
import and exception checks pass. Installed checks pass 63 cases on RTX and
63 on Lavapipe, zero skips, with both changing-color captures passing.

Fresh smoke captures pass nine pairs (18 PNGs), and all 37 canonical catalog
pairs (74 PNGs) pass bounded semantic assessment. Every finite 30-step and
completed-capture state matches; full source, executable and resolved-library
maps remain unchanged before/after (60 OSG and 56 Filament libraries). Actual
review covers all seven catalog sheets, nine smoke original comparisons, and
Fetch, Atlas, Hubo, operational-space arm and soft-body originals. The former
Fetch P2 is closed: restored gray material detail remains readable, with zero
pure-black and zero maximum-RGB <= 5 pixels in the original robot ROI, versus
1,038 and 3,352 at depth-final. State hash remains `66e67360bfaa5921`.
Assessment packet:
`/tmp/dart-filament-visual/qualified-catalog-specular-final/assessment-packet-index.json`.
Fresh supplement assessment covers DEPTH, opaque texture alpha, imported
specular shadows, axis-facing transparency and headlight shadows:
`/tmp/dart-filament-visual/qualified-supplement-specular-final/semantic-assessment.json`.

All 16 fresh functional benchmark arms pass. Their manifest records source,
executable, helper and core library identities; the catalog records the complete
resolved-library maps. The GPU ten-minute observation has completed with
functional and independent audit guards passing. The first CPU attempt is
rejected after external termination indicated by exit 143, before its final
record; the exact sender/cause is unknown and no renderer-crash diagnostic is
present. The fresh CPU retry completes 31,728 frames in 600.006953806 seconds;
all functional guards, 600 whole-phase RSS samples and full before/after
source, binary, 50 resolved-library, helper and ICD identities pass. CPU RSS
oscillates within 235,648..251,748 KiB; the ceiling during elapsed 60–120 seconds
is 251,744 KiB, and later ceilings are at most 251,748 KiB. Positive fitted slopes
alone do not
establish an ongoing leak.
Independent CPU audit passes the bounded identity and functional guards;
no leak-freedom or no-regression claim is made.
Unrelated builds including task_10 were active, so these observations do not
establish comparative timing or release resource acceptance. Accepted
performance, final acceptance reviews and native Windows/macOS CI remain. The
maintainer plans a 30-minute quiet window; no accepted timings have been launched,
fresh task_6 Release/Gazebo and scratch performance builders remain visible.

| Behavior | Implemented evidence | Recorded qualification |
| --- | --- | --- |
| DART primitives, float heightmaps, convex hulls | Descriptor dispatch, finite meshes, exact picking surfaces and rendered shape gallery | GPU/CPU fixtures and bounded canonical catalog assessment pass |
| Mesh hierarchy, scale, colors, UVs, alpha and textures | Descriptor fixtures; rendered hierarchy/alpha, JPEG/no-UV, black-border mipmaps, texture origin and normal-map cases | GPU/CPU pass; diffuse/vertex glTF linear-provenance observation remains recorded |
| Legacy gloss, specular, ambient and emission | Unchanged numeric shading contrasts; raw coefficient regression against actual OSG emission blue 51; real Fetch and imported shadow recovery | GPU/CPU pass; Fetch P2 closed for its canonical view, shadows retained; corrected Filament emission blue 49, prior conversion blue 2 |
| Point modes, connected lines, widths and voxel sizes | Exact picking geometry, visible live line edits, default-blue OSG comparison and HALF4 dark-color precision | GPU/CPU and bounded canonical assessment pass; opaque texture-alpha invariance is exact full RGBA |
| Soft meshes and live shape/color/version edits | Persistent buffers, invalidation, deformable rendered edits and texture resource churn | GPU/CPU and bounded canonical assessment pass; translucent interiors remain readable |
| Frame hierarchy, add/remove and shared/multiple worlds | Ownership and mutation cases; independent shadow masks and shared-world step ownership | GPU/CPU pass |
| Refresh/step callbacks, simulation permissions and real time | Viewer callback/budget cases, actual ImGui refresh and combined capture | GPU/CPU pass; one refresh and no physics step per paused capture |
| Camera, native presentation, resize and DPI | Camera/scale validation, automatic/override scale APIs and native orientation/resize/teardown | Prior Linux qualification passes; current default-window recovery and both RTX native fixtures pass; native Windows/macOS functional CI pending |
| Wireframe, headlight, displayed nonlinear depth and shadows | Body wireframe, depth capture, camera-following headlight, normals and independent world shadows | GPU/CPU regressions, real Fetch assessment and fresh DEPTH/headlight-shadow supplements pass |
| ImGui and debug overlays | Draw offsets/scales/textures/callbacks; labels, outlines, occlusion, custom font, grid and support polygon | GPU/CPU pass; overlay transitions and PNG/raw equality pass |
| Picking, constrained dragging, IK and custom handlers | Nine interaction cases plus Viewer input routing, modifiers, lifetime and exception cases; shared demo factories | GPU/CPU fixtures pass; bounded static canonical assessment passes; live interaction and native CI remain separate |
| PNG, recording and full RGBA | Orientation, physical dimensions, write recovery, pause/resume/restart, queued single-consume and combined capture | GPU/CPU/native pass; exact PNG/raw/recording pixels and refresh counts verified |
| C++/Python demos, tutorials and capture tooling | Same 37 C++ factories; then backend-selectable Python hosts and ownership/capture regressions | Both C++ activation cycles and all 37 bounded canonical pairs pass; CP314 65 focused checks pass on both devices, prior full suite 514 passes |
| Installed C++17 shared/static GUI | Relocated aggregate public-header consumers and exact 21 license hashes | Final shared and source-SDK static consumers pass |
| Wheels and renderer-free imports | Actual CP313 source-SDK repair, ABI, notices/RECORD, isolated lazy import and exception smoke | Current wheel ABI/notices/import and installed 63 cases per device plus full RGBA changing-color captures pass |
| Windows/macOS native rendering and package closure | Mandatory standard-runner functional workflow; CI-only Windows Mesa WGL | Prepared and unpublished; actual native CI pending approval |
| Linux speed, setup, full RGBA and resources | Completed-frame driver, measured OSG noise and immutable provenance runner | Fresh 16 functional arms and independent GPU/CPU observation audits pass; first incomplete CPU attempt rejected; accepted comparisons pending |
| Physics/core and Gazebo | Isolated GUI dependencies, unchanged physics/parsers, default tests and patched forward lane | 168 default CTest entries and patched forward checks pass; no unpatched last-release ABI claim |

Production tone mapping can still map near-black colors to zero; the precision
regression isolates storage using LinearToneMapper and does not claim exact
production dark-color parity. Paired visual assessment must distinguish this
from lost geometry or unreadable annotations. glTF linear diffuse/vertex
provenance differs from the retained display-color boundary; no reproduced OSG
appearance defect has established a release blocker for that observation.

Canonical images retain some neutral-surface brightness, shadow softness, glyph
metric and wire triangulation differences. Shared clipping and small subjects
limit assessment outside the visible framing; optional ssik is unavailable and
both sampled states contain zero skeletons. Static views do not qualify live
input, interactive DPI, alternate cameras/modes or later dynamics. The Fetch
resolution and supplement reports preserve these limits and image hashes.

The preview plan held its default switch until every gate passed; that staged
implementation plan is superseded by complete replacement. Equal/better and
release-ready claims still require all acceptance gates. Windows/macOS performance is maintainer-deferred;
functional native rendering and packaging remain required.
