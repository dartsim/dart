# dartsim GUI Toolkit and Language Decisions

## Status

Proposal. This document owns the durable rationale for the `dartsim`
application's GUI toolkit (Dear ImGui + GLFW vs Qt), application-shell language
(C++ vs Python), the Filament headless/rendering-pipeline scope, and the
distribution plan. Operating state for the workbench lives in
`docs/plans/dashboard.md` (PLAN-101) and
[`../plans/101-dartsim-gui-simulator.md`](../plans/101-dartsim-gui-simulator.md);
durable application architecture lives in
[`dartsim_gui_simulator.md`](dartsim_gui_simulator.md). This file owns the
decision plus its evidence; it does not repeat operating status.

The verdict and triggers below were shaped by independent specialized review
(four landscape/technology research passes, plus an architecture review and a
critique/steelman review). Their corrections — most importantly that the
existing UI seam bounds renderer swaps but not a retained-toolkit migration —
are folded into the decisions, the costed exit, and the risk register.

A dated re-investigation (2026-05-28) **reaffirmed Decision 2** and refreshed
several stale figures; see the addendum immediately below before reading the
original decisions. Where the addendum and the original body disagree on
numbers, the addendum is current.

## Re-investigation addendum (2026-05-28)

A fresh pass (codebase re-measurement against the live tree + 2025–2026 external
evidence + adversarial re-checking of this doc's load-bearing claims; full memo
in `.omc/research/gui-toolkit-reinvestigation-2026-05.md`) **reaffirms
Decision 2: keep GLFW + Dear ImGui; do not migrate to Qt.** None of the four Qt
triggers has fired. The fresh evidence strengthens the verdict rather than
weakening it. Three corrections to the body below:

1. **The costed-exit figures are stale and understate the rewrite surface.**
   `editor.cpp` was 632 lines when the exit table was written; it is now **1,150
   lines on `main` and 1,589 on the in-flight editor branch** (PR #2716), with
   13 immediate-mode panel builders (not ~10). Total `dartsim/ui` is **5,722
   lines on `main` / 10,109 on the branch** across 12 modules — versus the body's
   "~1,250-line action layer" framing. The toolkit-free action _verbs_ still
   dominate and still survive a Qt move, but the immediate-mode view surface that
   would be **rewritten** for a retained toolkit (`editor.cpp` + per-frame view
   binding such as the every-frame `OutlinerRow` re-flatten) is now ≈2,500 lines,
   ~4× the body's implied ~632. The qualitative ranking (viewport + model/view
   rewrite dominate cost) holds; the absolute numbers were optimistic. **This
   reinforces "don't migrate" on cost while confirming the deferral is accruing
   uncapped rewrite debt.**

2. **The "no production reference for a Filament↔Qt viewport" risk is
   outdated.** Cascadeur 2026.1 ships a Qt/QML DCC whose scene viewport was moved
   to Filament. This de-risks a future spike; it does not fire a trigger. The
   _narrow_ claim — no public reference for the zero-copy offscreen-texture
   composite specifically — may still hold (Filament external-texture import
   remains unplanned beyond Android), so a spike must still settle the
   native-child-window vs offscreen-composite question.

3. **Decision 2's Action 1 (the CI guard) was never enforced on `dartsim/ui` —
   now fixed.** `scripts/check_api_boundaries.py` previously covered only
   `python/dartpy`. It now also forbids ImGui/GLFW/Filament symbols, headers, and
   references in `dartsim/ui` (check id `dartsim-ui-backend-leak`, wired into
   `pixi run lint`), and the two leaked header comments + one CMake comment were
   cleaned. The boundary is no longer discipline-only.

**Trigger status (2026-05-28):** none firing. Closest to becoming observable is
the **large-tree jank** trigger — the per-frame outliner re-flatten has grown
with the view layer while no scene yet reaches ~5–10k nodes; this is the trigger
the growing view-debt makes most worth preempting with a measurement. The
**accessibility/i18n** trigger remains the only structural one (ImGui's a11y
issue #8022 and RTL/shaping gaps are confirmed permanent, not roadmap items) and
remains unfiled. ImGui v1.92's dynamic-fonts/texture work removed two real
DPI/atlas objections; the docking branch is still unmerged but
maintainer-recommended (pinning per Decision 4 still correct).

**Recommended next action (highest-leverage, lowest-regret):** the CI guard
above (done) and the Filament offscreen render-to-texture spike (done — see the
result below). **Do not start a Qt port.**

### Offscreen-viewport spike result (2026-05-29)

The Decision-3 prerequisite — the "hard ~80%" of both a Qt viewport and a
web/streaming viewer — was executed. **Result: the offscreen render-to-texture
path works with on-screen parity.**

- **Phase 1 (parity gate) — PASS.** Rendering the live dartsim scene to an
  offscreen Filament `RenderTarget` backed by an RGBA8 color `Texture` matches
  the swapchain render to within `maxDelta=2` of 255 over a 1280×720 `--demo`
  frame (tolerance 4), with non-trivial content. Implemented as an env-gated
  diagnostic in `dart/gui/detail/offscreen_parity.cpp` and runnable via
  `pixi run gui-offscreen-parity` (OpenGL/llvmpipe headless). So the renderer can
  produce a correct texture, not just a window surface — this de-risks the
  rendering half of any composited front-end (Qt or web) and directly unblocks
  headless sensor cameras and a future `rerun`/MeshCat-style stream off the
  shared pipeline (Identity B), which only need render-to-texture + CPU readback
  at modest rates.

- **Phase 2 (host-composite strategies).** Two ways to get Filament pixels into
  a Qt window, with the spike's evidence:

  |             | Strategy A — native child window                                                                                                           | Strategy B — offscreen texture composite                                                                                                                                                                                                                     |
  | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
  | How         | Filament `createSwapChain(nativeWindow)` into a Qt-owned native handle (`QWidget::createWindowContainer` + `winId()`/`QWindow::fromWinId`) | Filament renders to a `Texture` (proven), host samples it in its own GL/Vulkan context (`QRhi`/`QQuickFramebufferObject`)                                                                                                                                    |
  | Maturity    | Reuses the exact path the live viewport already uses; lowest risk                                                                          | Render-to-texture proven here; **zero-copy GPU sharing into the host context is the unproven part** — Filament external-texture export is unplanned beyond Android, and a per-frame `readPixels` CPU roundtrip is too slow for a 60 fps interactive viewport |
  | Compositing | Opaque native child window: Qt widgets/popups/transparency do not compose over it cleanly                                                  | Full Qt compositing (the texture is a normal QML/widget item) — the reason to want B                                                                                                                                                                         |
  | Verdict     | Low-risk fallback if a Qt viewport is ever built; accepts compositing limits                                                               | Higher fidelity but gated on Filament GPU-texture interop that does not exist today                                                                                                                                                                          |

  **Recommendation:** if a Qt viewport is ever pursued, start with Strategy A
  (low-risk, reuses the shipping path) and treat Strategy B as blocked until
  Filament exposes cross-context texture sharing on desktop.

- **Phase 3 (Qt-trigger implication).** This **informs** the Decision-2 Qt
  trigger; it does not fire it. The rendering prerequisite is no longer a
  blocker — it is proven low-risk — so the residual Qt risk narrows to the
  **interactive compositing handoff** (Strategy B's missing GPU interop) plus the
  unchanged structural triggers (a filed accessibility/i18n requirement; measured
  large-tree jank; a funded downstream need). The verdict stands: **keep GLFW +
  Dear ImGui; do not migrate to Qt.** The lasting win is the proven offscreen
  pipeline, which serves headless/sensor/web regardless of the toolkit outcome.

## Purpose

The `dartsim` application (north-star consumption mode 3) currently uses GLFW3
for windowing, Dear ImGui (docking branch) for widgets, and Google Filament for
rendering, with the renderer and toolkit hidden behind `dart::gui`. This
document re-examines, with 2025–2026 evidence, three questions the design doc
previously settled only as revisitable non-goals:

1. Is GLFW + Dear ImGui the right north star, or should `dartsim` adopt Qt (with
   Filament driven as a headless render pipeline composited by Qt)?
2. Should the application shell be C++ or Python?
3. What do industry trends and the distribution plan imply for both?

The goal is to tie the choice to `dartsim`'s actual mission and users,
benchmarkable triggers, packaging realities, and the existing backend-hidden
boundary — not to toolkit fashion. Scope is the `dartsim` **application** only.
The library and bindings (modes 1–2: `dart`, `dartpy`) are out of scope; their
language is already C++ + nanobind.

## Decision Model (mission-first)

Choose the toolkit and language from `dartsim`'s mission and users, not from what
incumbent simulators happen to use. From the north star and PLAN-101, `dartsim`
exists to (1) serve DART's **easy start** — a researcher reaches a working
simulation fast — and (2) act as a **forcing function** validating the
experimental World toward DART 7 promotion. Its users are **researchers and developers**,
not non-programmer end users. Each decision weighs: user fit, capability ceiling
versus _actual_ needs, integration risk with Filament, cross-OS deployment
reliability, maintenance/bandwidth cost, and reversibility behind the existing
`dart::gui` seam. This mirrors the workload-first model in
[`scalable_compute_decisions.md`](scalable_compute_decisions.md): decide from
evidence and keep backend choices behind a neutral boundary.

## The Strategic Fork (read this first)

The honest finding from the landscape survey is that "ImGui vs Qt" is the _second_
question. The field has split into three camps, and `dartsim` must first decide
which one it is in:

| Camp                             | Examples                                                            | GUI + language                                                                                | What it optimizes                         |
| -------------------------------- | ------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- | ----------------------------------------- |
| Established desktop robotics DCC | Gazebo, Webots, CoppeliaSim, RViz                                   | **Qt + C++** (CoppeliaSim _migrated to_ Qt)                                                   | Rich interactive authoring for humans     |
| GPU/game-engine tools            | NVIDIA Omniverse (`omni.ui` is built on Dear ImGui), rerun (`egui`) | **Immediate-mode + C++/Rust**                                                                 | Programmer-facing tools over a big engine |
| ML/RL frontier                   | Isaac Lab, MuJoCo MJX, SAPIEN, Genesis, Newton, Brax                | **Python-first + GPU, headless**, web viewers (Foxglove, rerun, MeshCat) for telemetry/replay | Throughput, batching, learning loops      |

This produces a genuine north-star fork for mode 3:

- **Identity A — interactive desktop authoring DCC.** Design/edit/run/record/
  replay with scene tree, inspector, gizmos, picking, undo/redo. This is what is
  built today. It requires a desktop GUI and a C++ shell. Web viewers cannot
  author.
- **Identity B — visualization surface for Python-driven research.** A thin
  Python/Jupyter driver plus a headless Filament render service streaming to a
  web/`rerun`-style viewer. Rides field momentum, reuses the already-working
  headless pipeline, and sidesteps both the docking-branch and the Filament↔Qt
  risks — but it is a _viewer_, not an authoring editor.

**Recommended resolution: stay Identity A, because DART already has mode 2.**
`dartpy` already serves the Python research wave. Collapsing mode 3 into
Python + web-viz would largely reproduce "`dartpy` + an off-the-shelf viewer,"
which users can already assemble — so mode 3's _differentiated_ reason to exist
is exactly the authoring/easy-start DCC experience that B does not provide. The
large sunk investment already made (object/selection/command/undo-redo/scene-IO/
record-replay engine) is authoring value that B would discard. **The synthesis
that avoids a false choice:** build the headless Filament pipeline (Decision 3)
as a _shared_ rendering capability so the same renderer can later stream to
Python/web for mode-2 visualization. That hedges toward B without a web rewrite,
and it is independently the highest-leverage rendering investment.

This fork is consequential; it was put to the project lead, who adopted the
recommended direction (Identity A) as the working north star. It remains
revisitable via the Decision 2 triggers and open question Q1 below.

## Decision 1 — Application shell language: C++, with embedded Python scripting

**Verdict: the `dartsim` shell is C++. Not close. Python belongs as an embedded
scripting/automation/REPL layer, never on the render/step loop.**

Rationale:

- **Filament has no Python bindings and none are planned.** The renderer
  integration alone forces C++ for the viewport and frame loop. A Python shell
  would require hand-maintained Filament wrappers against a fast-moving API — a
  standing liability.
- **Industry is unanimous:** Blender, Houdini, Maya, Unreal, Isaac Sim/Kit, and
  Gazebo all use a C++/C core that owns the real-time loop with Python (or Lua)
  as the scripting layer. CoppeliaSim runs Python **out of process** (sockets)
  because in-loop Python was too costly — a direct warning against a Python
  frame pump.
- **Deployment:** a C++ binary plus shared libs via conda/pixi is far more
  reliable cross-OS than PyInstaller/Briefcase (which cannot cross-compile);
  interpreter startup and GC/GIL jitter also work against a 60+ FPS interactive
  loop.
- **Repo precedent agrees.** [`compute_backend_research.md`](compute_backend_research.md)
  already records as durable policy that a Python-first kernel/array framework
  "would invert a C++ engine's dependency direction and is better treated as a
  reference than a foundation." The same logic applies to the app shell.

The GIL itself is not the core problem (native code releases it); the cost is
per-frame/per-object boundary crossings and latency jitter. nanobind crossings
are ~10× cheaper than pybind11, but the rule is "minimize crossing count," not
"pick a binding library." Python 3.13 free-threading (~40% single-thread
penalty, ecosystem not certified) is not a safe 2025–2026 bet.

| Concern                                                        | C++ shell     | Embedded Python |
| -------------------------------------------------------------- | ------------- | --------------- |
| Render/step loop, Filament resources, viewport                 | **Yes**       | No              |
| Scene scripting, parameter sweeps, batch/headless automation   | shared API    | **Yes**         |
| Extension hooks (e.g. `on_step_end` with a read-only snapshot) | provides      | **Yes**         |
| In-app REPL / console                                          | hosts CPython | **Yes**         |

**Where Python adds the most value:** an embedded CPython interpreter exposing a
_coarse-grained_ scripting API (create object, get/set property, run N steps,
snapshot state), reusing the `dartpy` nanobind idiom, run on its own thread so
the render/step loop never waits on it. This is the correct scope for the
Console's reserved Python REPL.

## Decision 2 — GUI toolkit: keep Dear ImGui + GLFW now; actively hedge Qt

**Verdict: keep GLFW + Dear ImGui for now. Do not migrate to Qt speculatively.
But the deferral must be _active_, not passive — because the existing UI seam
does NOT bound a Qt migration the way it first appears.**

### What the seam actually buys (corrected by review)

`dartsim/` contains zero ImGui/GLFW/Filament code; panels are built against the
abstract `dart::gui::PanelBuilder`, and the editor logic lives in toolkit-free
action/view-model files. That is real and worth protecting — but `PanelBuilder`
is **immediate-mode to the bone** (`button()` returns a click, `slider(label,
&value, min, max)`, per-frame `build(PanelBuilder&)` closures, `OutlinerRow`
snapshots re-flattened every frame). Qt is **retained** model/view
(`QAbstractItemModel`, signals/slots, delegates, lazy population). You cannot
host a `QTreeView` behind a per-frame `collapsingHeader()` call without throwing
away the virtualization, selection model, and lazy children that are the entire
reason to adopt Qt.

So the precise truth: **the seam bounds immediate-mode _renderer_ swaps and
preserves the engine + the action "verbs"; it does NOT bound adoption of a
retained toolkit.** The earlier "Qt is a bounded View-layer change" framing was
too generous and is corrected here.

### Costed exit (so the deferral is an option, not faith)

| Survives a Qt move                                                                                                                                                                                                                     | Rewritten/built for Qt                                                                                        |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| The entire `dartsim_engine` (object/selection/command/undo-redo/name/sim-control/record-replay/scene-IO)                                                                                                                               | `editor.cpp` (632 lines, ~10 immediate-mode panel builders)                                                   |
| The ~1,250-line toolkit-free action layer (`outliner_actions` 476+154, `simulation_actions` 152+88, `viewport_actions` 148+89, `project_actions` 79+72) — the verbs (`selectOutlinerObject`, `commitOutlinerRename`, command dispatch) | The `PanelBuilder` abstraction + its ImGui impl                                                               |
| The Filament render core in `dart/gui/detail/`                                                                                                                                                                                         | A new retained model/view adapter (e.g. `QAbstractItemModel`) replacing the per-frame `OutlinerRow` snapshots |
|                                                                                                                                                                                                                                        | The Filament↔Qt viewport composite (no production reference — the spike, see Decision 3)                      |

The dominant cost and risk is the **Filament↔Qt viewport** and the **model/view
rewrite**, not the editor logic. Budget a multi-week single-developer effort
gated on a successful viewport spike — i.e. months once the spike lands, not
weeks.

### Make the hedge active (three concrete actions)

1. **Add a CI guard** forbidding ImGui/GLFW/Filament symbols (and headers) in
   `dartsim/ui/`. Today the boundary is discipline-only, and discipline already
   slipped (a stray "ImGui" comment in `outliner_actions.hpp:102`). DART already
   runs API-boundary inventory checks and `check-docs-policy`; this is the same
   pattern. Without it, the action layer rots and the deferral quietly becomes a
   one-way door.
2. **Time-box a Qt + Filament model/view spike** as a _dated_ decision input
   (not an open-ended trigger): prove the two genuine unknowns — (a) a Filament
   viewport composited inside Qt (offscreen render-to-texture; see Decision 3),
   and (b) a `QAbstractItemModel` over the engine's scene model driving a
   virtualized tree. "Defer until you need it" for a foundational toolkit is how
   projects end up porting under deadline (CoppeliaSim's lesson). The spike keeps
   the option real without paying for the full migration.
3. **Name the concession.** Staying ImGui cedes the _polished, accessible,
   non-programmer DCC_ ground to Gazebo/CoppeliaSim, and immediate-mode view code
   keeps accruing as rewrite debt. That trade is acceptable because researchers
   are not that audience — but it is a deliberate cede, and action (1) keeps the
   debt from metastasizing past the view layer.

### Licensing is not a blocker (recorded so it is not re-litigated)

For a freely-distributed open-source executable shipped via package managers,
**LGPLv3 Qt is fine**: dynamic linking (which conda-forge's shared-lib Qt builds
do by default) satisfies the relinking obligation; the static-link trap and the
iOS no-dynamic-loading issue do not apply to a desktop conda/apt/homebrew tool.
Dear ImGui (MIT) is simpler, but licensing is not a reason either way.

### Qt triggers (observable; re-decide when one fires)

- A **filed** accessibility/i18n/RTL requirement (institutional, grant, or
  compliance). ImGui cannot meet this structurally (no accessibility tree by
  design; partial i18n/IME/shaping).
- The outliner/tables routinely exceed **~5k–10k variable-height nodes** with
  _measured_ `ImGuiListClipper` jank.
- A **third-party packager or funded downstream** asks for Qt/native desktop
  integration.
- The spike (action 2) shows the Filament↔Qt composite is _low_ risk _and_ a
  concrete product need exists.

## Decision 3 — Filament rendering/headless pipeline: invest, sequenced as the Qt prerequisite

**Verdict: invest in Filament's offscreen/headless render-to-texture and
multi-view/camera path now. It is independently valuable AND it is the
prerequisite spike for any future Qt viewport — making it the lowest-regret
rendering investment regardless of the toolkit outcome.**

What is confirmed (API-level and in-repo):

- Headless is first-class: `createSwapChain(w, h)` or a `RenderTarget` of
  `Texture` attachments + `renderStandaloneView`; `Renderer::readPixels` already
  used in `dart/gui/detail/screenshot.cpp`; a `--headless` path and a `noop`
  backend already run in CI screenshot smokes.
- Multi-view/multi-camera per frame is native (multiple `View` + `Camera` +
  `RenderTarget`); a handful (a few sensor cameras + a UI view) is the intended
  usage.
- Physically-based camera (aperture/shutter/ISO) exists for synthetic cameras.

Scope it honestly:

- **Camera simulation is interactive/modest-rate** (≈5–30 Hz, 1–4 cameras).
  Depth needs manual metric conversion (reversed-Z); there is **no first-class
  G-buffer/segmentation** (custom MRT materials only); `readPixels` is a GPU→CPU
  DMA the docs flag as costly. Filament has **no CUDA/GPU-tensor path**, so it is
  **not** an Isaac/SAPIEN-class batched synthetic-data engine — that workload is
  mode-2 territory and belongs to the compute roadmap (PLAN-030), not the
  renderer.
- **The live viewport currently renders to `createSwapChain(nativeWindow)`** — a
  native GL/Vulkan surface, not an offscreen texture. A Qt front-end needs the
  offscreen→texture→composite path instead. So building the offscreen pipeline
  now _is_ the hardest 80% of a future Qt viewport: headless-first is sequenced
  with Decision 2, not orthogonal to it.

## Decision 4 — Distribution / deploy plan

- **Ship a C++ single binary + shared libs via conda/pixi** (already the stated
  model). Reliable cross-OS; Python desktop packaging is fragile and cannot
  cross-compile.
- **Pin the docking-ImGui build for the `dartsim` distribution.** Today the
  shared build flips between system ImGui and the docking branch on demand
  (documented in [`dartsim_gui_simulator.md`](dartsim_gui_simulator.md), the
  "Docking workspace" note); end users should never hit that reconfigure dance.
- **Harden the Filament per-OS dependency and backend story.** Materials are
  compiled for **OpenGL and Vulkan only** (`matc -a opengl -a vulkan`), and
  `metal`/`webgpu` are intentionally rejected (`render_context.cpp` `parseBackend`);
  the build defaults to OpenGL. Consequence: **macOS has no Metal material path**
  and runs via OpenGL (Apple-deprecated) or MoltenVK-Vulkan. The packaged macOS
  story (and the pinned-Filament-archive-vs-`Filament_ROOT` split across
  platforms) needs an explicit owner before mode 3 is promoted as cross-platform.
- **Keep the binary lean.** This is a standing advantage of ImGui and a reason
  not to adopt Qt without a trigger (Qt adds bundling weight; manageable via
  conda shared libs, but real).
- **Web/streaming viz is a complement, on demand.** If researcher demand for
  remote/headless visualization appears, a `rerun`/MeshCat-style streaming path
  off the shared headless pipeline (Decision 3) rides the field's momentum far
  better than a Qt desktop rewrite — and is the concrete bridge toward Identity B
  without abandoning Identity A.

## Risk Register

| Risk                                                                                                         | Severity                     | Note / mitigation                                                                                                                                                                                                                                                                          |
| ------------------------------------------------------------------------------------------------------------ | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Filament project risk** (Google could deprecate, as with Sceneform)                                        | High                         | Health is currently excellent (weekly releases, 6-platform CI), but robotics adoption is ≈zero — `dartsim` is trailblazing, with no peer to share maintenance. The backend-hidden boundary is the mitigation: a renderer swap is the _one_ thing the seam genuinely bounds. Keep it clean. |
| **Filament has no GPU-tensor path**                                                                          | Medium                       | Permanent ceiling on Isaac/SAPIEN-class synthetic data; set expectations and route that workload to mode 2 / PLAN-030.                                                                                                                                                                     |
| **macOS/Metal material gap; Vulkan-headless historically buggy**                                             | Medium                       | OpenGL-headless is the proven path; macOS deployment needs explicit validation (Decision 4).                                                                                                                                                                                               |
| **ImGui docking branch is unmerged & self-described as needing a rewrite; build flips system/docking ImGui** | Medium                       | Active build fragility today, not a deferred risk. Pin the docking build (Decision 4); track upstream.                                                                                                                                                                                     |
| **Accessibility/i18n appear nowhere in current docs/plans**                                                  | Medium (latent)              | A future institutional/grant requirement ImGui cannot meet — the most likely real Qt trigger. Filed as a trigger, not assumed away.                                                                                                                                                        |
| **Filament↔Qt has no production reference**                                                                  | High (only if Qt is pursued) | Gate any Qt move on the Decision-2 spike.                                                                                                                                                                                                                                                  |
| **Maintainer bandwidth / bus factor**                                                                        | Medium                       | Owning pinned-docking-ImGui + bleeding-edge Filament (+ possibly Qt) is a lot for a research team. Favors the lean ImGui path and the CI guard that keeps the seam cheap to maintain.                                                                                                      |

## Open Questions And Decision Triggers

- **Q1 (north star): mode 3 is an interactive desktop authoring DCC (Identity A)
  — adopted as the working direction.** It stays revisitable: if real usage
  shifts toward Python-driven research visualization (Identity B), reopen this;
  the shared headless pipeline (Decision 3) keeps that pivot cheap.
- Does a filed accessibility/i18n requirement appear? → re-decide Qt.
- Does the outliner exceed ~5k–10k variable-height nodes with measured jank? →
  re-decide Qt.
- Does the Decision-2 spike show a low-risk Filament↔Qt composite _and_ a product
  need? → re-decide Qt.
- Does Filament's upstream health regress? → exercise the renderer-swap seam.

## Related Designs

- [`dartsim_gui_simulator.md`](dartsim_gui_simulator.md) — durable application
  architecture (this doc owns the toolkit/language rationale its Non-Goals
  reference).
- [`scalable_compute_decisions.md`](scalable_compute_decisions.md) and
  [`compute_backend_research.md`](compute_backend_research.md) — the
  decision-model and "Python inverts the dependency direction" precedents this
  doc applies.
- [`filament_fidelity_profile.md`](filament_fidelity_profile.md) and
  [`renderer_realtime_and_scalability.md`](renderer_realtime_and_scalability.md)
  — the renderer-neutral camera-sensor seam and real-time/offline loop that own
  the Decision 3 rendering details this doc only scopes.
- `docs/plans/101-dartsim-gui-simulator.md`, `docs/plans/dashboard.md`
  (PLAN-101) — operating state.
- `docs/onboarding/gui-rendering.md` — the backend-hidden renderer policy and the
  `dartsim` editor overview.

## Sources

Landscape: NVIDIA Isaac Sim/Lab and Omniverse Kit (`omni.ui` on Dear ImGui);
MuJoCo `simulate` + MJX; Gazebo `gz-gui` (Qt+OGRE); Webots (Qt+wren);
CoppeliaSim (Qt, migrated from a custom GL UI); PyBullet; Drake Meldis/MeshCat
(browser/three.js); SAPIEN/ManiSkill (Vulkan); Genesis; NVIDIA Newton
(Warp/USD); Brax; RViz/RViz2 (Qt+OGRE); Foxglove Studio; rerun.io (Rust egui +
wgpu); Blender. Toolkit: Dear ImGui FAQ and accessibility issue (#8022); ImGui
docking-branch status (#2109); Qt LGPL obligations; conda-forge Qt shared-libs;
Qt Advanced Docking System / KDDockWidgets. Filament: `Engine.h`/`Renderer.h`/
`View.h`/`RenderTarget.h`/`Camera.h`; offscreen rendering and `readPixels` issues
(#119, #1315, #2007, #2011); release history. Language: nanobind vs pybind11
benchmarks; Blender/Houdini/Maya/Unreal/Isaac/CoppeliaSim scripting
architecture; PyInstaller/Briefcase/conda packaging; Filament language-support
(no Python bindings). In-repo evidence: `dart/gui/panel.hpp`,
`dart/gui/application.hpp`, `dart/gui/detail/{render_context,screenshot,
backend_sources.cmake}`, `dartsim/ui/**`.
