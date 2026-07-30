# Agent Simulation Verification: Research and Design Record

Durable rationale for DART's agent-facing 3D-scene and physics verification
tooling. The how-to lives in
[`../onboarding/agent-sim-verification.md`](../onboarding/agent-sim-verification.md)
and the `dart-verify-sim` skill; this doc records _why_ the tooling is shaped
the way it is, backed by measured evidence.

## Problem

DART's domains — articulated-body dynamics, collision, robotics — require
state-grounded 3D understanding. Early language-only agents could not inspect
images natively; current image-capable agents can, but pixels still do not
expose solver state and visual physical reasoning remains weaker than measured
state. An agent therefore needs to inspect a scene, run a simulation, and
decide whether the result is correct, headlessly and without hidden context.
There is no established prior art for this exact task: the literature splits
into VLM physical-reasoning benchmarks (which show models are weak at physics
from images — e.g. PhysBench reports SOTA VLMs near 40% vs ~96% human) and
engine self-testing (CPU/GPU parity, conservation as a diagnostic). Neither is
"an agent audits an engine's output," so DART's tooling is built conservatively
and self-grounding rather than mimicking an authority that does not exist.

## North-star capability contract and merged lineage

The original goal was not merely "take a screenshot." A fresh agent must be
able to discover what is in a world, measure what the simulation did, choose
and validate a useful view, inspect the actual pixels, reconcile the visible
observation with the measured state, and publish claim-tied evidence that a
later agent or human can reproduce. Rendering and geometric reasoning belong
in the backend-hidden GUI core; Python and command-line tools orchestrate those
capabilities. This is the visual-verification instance of DART's AI-native
north star: important project understanding and verification must survive
agent and session boundaries as tracked evidence.

The 2026-07-29 audit reconstructed that contract from merged PR bodies, code,
tests, and durable docs:

| Capability layer                                                | Merged lineage                                                                                                                                                                                                 | Acceptance owned by the layer                                                                                                         |
| --------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| Reproducible maintained renderer and capture                    | [#2647](https://github.com/dartsim/dart/pull/2647), [#2687](https://github.com/dartsim/dart/pull/2687), [#2774](https://github.com/dartsim/dart/pull/2774)                                                     | Backend-hidden `dart::gui`, headless capture, and measured offscreen/render-target parity                                             |
| Debug-observability foundations                                 | [#2690](https://github.com/dartsim/dart/pull/2690), [#2984](https://github.com/dartsim/dart/pull/2984)                                                                                                         | Core debug descriptors, collision/contact inspection, world/provider seams, and visually verified renderer fidelity                   |
| Reproducible evidence workflow                                  | [#2992](https://github.com/dartsim/dart/pull/2992), [#3084](https://github.com/dartsim/dart/pull/3084)                                                                                                         | Capture packets/manifests and reviewable, reproducible visual evidence rather than local-only screenshots                             |
| Durable agent/project-management contract                       | [#2986](https://github.com/dartsim/dart/pull/2986)                                                                                                                                                             | Architecture findings, plans, work packets, and verification evidence survive tool and session boundaries through tracked docs        |
| Text correctness oracle                                         | [#2994](https://github.com/dartsim/dart/pull/2994), [#3313](https://github.com/dartsim/dart/pull/3313), [#3320](https://github.com/dartsim/dart/pull/3320)                                                     | Metrics, scene dumps/diffs, trajectories, contacts, and first-divergence evidence decide physics correctness                          |
| Agent-facing visual loop                                        | [#3313](https://github.com/dartsim/dart/pull/3313), [#3320](https://github.com/dartsim/dart/pull/3320)                                                                                                         | Headless images corroborate text; verdict, golden, sheet, A/B, and verification-bundle tools are reproducible                         |
| Adequate cameras, core debug layers, and claim-tied publication | [#3371](https://github.com/dartsim/dart/pull/3371)                                                                                                                                                             | Measured view adequacy, adaptive cameras, world-derived offscreen layers, deterministic capture sidecars, and claim cover/publication |
| Workflow routing and blocking end-to-end exercise               | [#3386](https://github.com/dartsim/dart/pull/3386)                                                                                                                                                             | Relevant agent workflows discover this capability and Linux CI exercises the real renderer and debug layers                           |
| DART 6 renderer parity                                          | [#3304](https://github.com/dartsim/dart/pull/3304), [#3314](https://github.com/dartsim/dart/pull/3314), [#3374](https://github.com/dartsim/dart/pull/3374), [#3385](https://github.com/dartsim/dart/pull/3385) | Bounded OSG/example capture, adaptive views/debug/evidence tooling, workflow routing, and required visual CI on `release-6.20`        |

The current model-upgrade follow-up closes three convergence gaps exposed by that
lineage audit: all descriptor bounds, viewport projection, and fit-distance
math now live in `dart::gui` and are bound into Python; capture sidecars name
representative start/middle/end sequence frames and the structured fields a
semantic reviewer must record; and evidence publication now persists that
text/image reconciliation and fails on a `fail` or `uncertain` semantic verdict.
Image-capable agents must open those artifacts and reconcile them with the text
oracle—machine pixel checks alone do not satisfy this contract.

This contract does **not** silently promote every renderer wishlist item into a
visual-verification blocker. Specular IBL, a depth-tested debug material, exact
ID/depth-buffer visibility, and solver paths without a stable contact-force key
remain explicit future renderer or solver work. They are limitations to name
when relevant, not evidence that the implemented text-plus-visual loop is
unfinished.

## Core decision: lead with text, corroborate with images

Two blind-judge A/B studies (fresh agent sessions given only an artifact plus
neutral questions, scored against a withheld key with clean controls) measured
how well each representation exposes seeded physics defects.

**Text (per-format detection over two rounds, 8 seeded defects):**

| Format         | Detection | False positives | Note                                                                                         |
| -------------- | --------- | --------------- | -------------------------------------------------------------------------------------------- |
| metrics series | 16/16     | 0–20%           | strongest single format                                                                      |
| trajectory TSV | 15/16     | 0–20%           | localizes which body/frame                                                                   |
| scene JSON     | 9/16      | ~10%            | only format that localizes _cause_ (gravity sign, joint-past-limit, missing collision shape) |
| contact log    | 2/16      | 0%              | decisive only for contact defects                                                            |

**Images (66 blind-judge cells, fixed per-scene camera):** dynamic, gross
defects (explosions, tunneling) are caught across layouts, but static geometry
defects — ground penetration and solid interpenetration — were missed by
_every_ representation. Single ~1280 px frames beat 4-frame filmstrips (worst:
25% detection, 33% control false-flags); UI panels measurably hurt
comprehension; resolution above ~1280 px showed no benefit at these scales.

**Conclusion.** Metrics and trajectories decide correctness; scene JSON
localizes structural causes; images confirm what a scene looks like and gross
dynamic failure but are not a trustworthy oracle for static geometry. The
tooling defaults follow: a text bundle (metrics + trajectory + scene JSON) as
the primary oracle, and a single UI-hidden ~1280 px 3/4-view frame (plus a
9-frame grid for motion) as corroboration.

Round-2 image studies should keep the same blind-judge protocol but score the
new camera and annotation arms explicitly: single 3/4 view, multi-view grid,
turntable samples, and Set-of-Mark-style annotations for body IDs, contact
markers, ground lines, and motion trails. `pixi run image-ab-round2 -- generate`
prepares a blinded local packet over real `dart.gui.render` captures and keeps
the answer key separate from the judge form; `pixi run image-ab-round2 -- score`
merges completed observations into the `pixi run image-ab-study` reducer. A row
set is research evidence only when it comes from real blind judgments over
seeded defects and clean controls. The default capture recommendation above
should change only after those measured deltas beat the current single-frame
baseline without increasing control false positives.

**Round 2 result (2026-07-06).** A fresh blinded DART 7 packet
(`study_id=agent-sim-image-ab-round2-20260706`, one visual judge, 16 rows) did
not justify changing the default image capture:

| Arm        | Detection | Delta vs single | False positives |
| ---------- | --------: | --------------: | --------------: |
| single     |       2/2 |             n/a |             2/2 |
| multi-view |       0/2 |            -2/2 |             1/2 |
| turntable  |       2/2 |             0/2 |             2/2 |
| annotated  |       2/2 |             0/2 |             1/2 |

Annotations reduced false positives relative to this small single-view control
but did not improve defect detection; multi-view reduced detection; turntables
matched single-view detection with the same high false-positive rate. The
result reaffirms the existing recommendation: keep images as corroboration,
prefer a single clean 3/4-view still by default, add grids only for motion
questions, and use metrics, trajectories, and scene JSON to decide correctness.

## Image-comparison tolerance policy

Golden-image comparison is opt-in, never a default-CI gate (renderer/driver
variance between llvmpipe and GPU-backed Filament makes strict pixel matching
flake-prone). The layered scoring adopted for the verdict tooling, drawn from
established practice: non-blank + report-only contrast → per-pixel diff with
anti-aliasing ignore and a Blender-style two-number budget (`--fail 0.016
--failpercent 1`) as the primary score → optional numpy SSIM and Habitat-style
relative-norm. Contrast is scene-dependent (a flat minimal render is not a
defect), so it is reported but only gates pass/fail on explicit opt-in. Goldens
are backend-specific and curated locally per backend, matching the repo's
`*.png` convention; the sidecar records the backend so a cross-backend run is
an explicit choice, not a silent flake.

## Conservation thresholds

No physics engine publishes a numeric energy/momentum drift pass/fail
threshold (MuJoCo endorses energy monitoring but gives no number). DART
therefore derives an empirical drift band from its own runs and uses a
bit-exact checksum mode for determinism regression (the precedent in
`tests/benchmark/integration/boxes_headless.cpp` and Avian's cross-platform
hashing), rather than importing an unfounded constant.

## Optional integrations

Of the visualization tools surveyed (rerun.io, meshcat, viser, Foxglove, k3d,
pyrender), only rerun.io is built and marketed for automated/VLM use
(programmatic data readback plus headless screenshots). It is adopted as the
single opt-in richer-inspection extra, behind a graceful fallback and never a
core or wheel dependency; the rest were rejected as human-in-browser or
redundant with DART's existing Filament/OSG paths.

For external model review, `pixi run verification-bundle` packages the text
oracle, one still frame, an optional grid, and a provider-neutral prompt into a
hash-recorded bundle. The helper deliberately stops at packaging: model choice,
API credentials, and review policy remain outside DART's default gates, and any
VLM answer must cite the bundled artifact fields it relied on.

## Dual-branch scope

DART 7 (`main`) uses headless Filament (offscreen render API, camera control,
image verdict/golden/sheet, text metrics/scene-dump/trajectory). DART 6
(`release-6.20`) has the OpenSceneGraph equivalent (GLX-pbuffer offscreen
helper, dartpy binding, ported image tooling) verified on a real X server;
conda-forge has no linux-64 Xvfb build, so DART 6 headless capture needs a real
display or a system-installed Xvfb.

## Active views, debug layers, and PR evidence (2026-07)

A review of the merged visual-verification lineage (#2690, #2836, #2984,
#2992, #3063, #3084, #3304, #3313, #3314, #3320) found four recurring gaps:
no PR ever embedded its visual evidence (bodies cite local `/tmp/*.png`
paths; GitHub user-attachments can only be minted in the web editor), cameras
were fixed enumerations with no notion of view adequacy, debug overlays never
reached the headless path, and artifacts were not tied to explicit claims.
The follow-up workflow closes these:

- **View adequacy is measured, not assumed.** `dart.gui.assess_view` scores a
  camera against the world geometrically (bounds-corner coverage, subject
  screen fraction, CPU pick-ray occlusion, screen-overlap ambiguity) and
  names issues (`cropped`, `too-far`, `too-close`, `occluded`, `ambiguous`);
  `select_viewpoints` deterministically searches a candidate grid and returns
  azimuth-diverse choices with recorded reasons. Geometry-first keeps
  assessment GPU-free and mirrors the text-first oracle policy.
- **Debug layers are world-derived and offscreen-capable.**
  `OffscreenRenderer` accepts a `DebugScene` through the viewer's unlit
  always-on-top overlay path (a plain render always clears it, so captures
  stay pure functions of their arguments); `dart.gui.debug_scene_for_world`
  composes named layers (frames, centers of mass, inertia, collision bounds,
  velocities, contacts from `world.collide()`, trajectory polylines,
  labels) that the built-in loop never aggregated. Labels are composited in
  Python because offscreen rendering has no UI text pass.
- **Evidence is claim-driven and rationale-carrying.** `evidence-select`
  performs greedy claim cover with redundancy pruning and size budgets and
  rejects artifacts that support no claim; `evidence-publish` renders the PR
  "Visual verification" section (environment, per-artifact what-to-observe,
  not-proven statements, limitations, repro commands). Media is
  GitHub-hosted, never committed: the default backend emits web-editor
  upload placeholders, and the `gh-release` backend uploads release assets
  but only behind `--yes` plus maintainer approval (release-asset images
  render inline in PR bodies; videos render as links — only
  user-attachments URLs get the inline player). Before any upload, publication
  recomputes artifact sizes and SHA-256 hashes plus claim coverage and pass
  state; any stale, tampered, incomplete, or inconsistent selection fails
  without a GitHub call. Release assets are content-addressed and never
  cross-content-clobbered; the publication manifest records each
  path/size/digest/URL binding so retrying a partial upload cannot mutate older
  evidence. Reusing an existing asset requires an exact GitHub-reported size,
  SHA-256 digest, and `uploaded` state; absent integrity metadata fails closed.
- **Case study (#2984 retrospective).** Re-verifying the renderer-fidelity
  PR with this workflow (`scripts/write_retro_2984_evidence_packet.py`)
  produced claim-tied evidence its own body declined to attach: a PBR
  metallic/roughness sweep, a default-vs-high-fidelity side-by-side with
  diff heatmap, and a world-derived debug-layer offscreen render — the
  "world-derived debug overlays" follow-up #2984 itself deferred. The
  view-adequacy loop caught a too-far framing before it shipped as weak
  evidence.

Known limits and future work: occlusion probes sample bounds corners (a
depth/ID-buffer readback would make visibility exact), thresholds are fixed
constants calibrated on primitive scenes, multibody/deformable layers reuse
only the transform-based subset, and inline PR video still requires the manual
user-attachments flow. The DART 6 adaptive-view, engine-debug-overlay, capture,
evidence, and required-CI port landed through #3374 and #3385; remaining branch
differences are backend-specific capability differences, not an in-flight
port.

Contact force arrows (DART 6 parity): DART 6 draws per-contact force arrows;
DART 7's `simulation::Contact` (`dart/simulation/body/contact.hpp`) is pure
collision-query output and carries no force. The solved per-contact impulses
persist post-step in the rigid contact scratch
(`m_contactScratch->problem.constraints`; `BoxedLcpContactSnapshot.f`), so the
first cut is now implemented as a bounded core exposure:
`World::getLastContactForces()` returns `ContactForce{point, force, bodyA,
bodyB}` captured at solve time (force = impulse / timeStep) for the rigid
SequentialImpulse and BoxedLcp paths, without ever mutating `collide()`
results. The GUI draws these as arrows through
`extractContactForceDebugLines` (reusing the DART 6 force color and
`contactForceScale`/clamps), wired into the `contacts` debug layer. Mapping
impulses to contacts for the unified AVBD/variational rigid path and for
multibody/deformable contacts remains deferred: those store per-row descent
state without a stable contact key, and forcing a force field onto the
query-time `Contact` would create a post-integration pose mismatch. For those
paths (and for replayed/rewound frames, which do not restore the transient
force list) the overlay draws contact points and normals but not force
magnitude.

## Core-first policy for agent visual tooling

Maintainer directive (2026-07): agent-facing visual debugging/verification
capabilities are implemented in the core rendering/GUI codebase — `dart::gui`
on DART 7, `dart::gui::osg` on DART 6 — and exposed through dartpy, never as
parallel Python reimplementations. Scripts and pixi tasks orchestrate core
APIs (scene setup, capture sequencing, claims/evidence workflow, GitHub
publication) but must not reimplement rendering, overlay drawing, text,
projection, or view-adequacy math. Rationale: agent needs are a driving
factor for core rendering/GUI capability, quality, and interfaces — a gap
found by agent tooling is fixed in the engine, where every consumer benefits.
Capabilities this policy has already driven into core: the offscreen
`DebugScene` channel with label compositing, world-aware debug extraction
(`extractDebugLines(World&)`, wiring the previously dead body-frame/COM
flags), the shared projection primitive (`projectToPixels`), core view
adequacy (`assessView`), world-space scene bounds (`sceneBoundingSphere`),
viewport-aware camera fitting (`fitOrbitCameraToViewport`),
`World::getRigidBodyNames`, and on DART 6 the `DebugOverlay` attachment
(always-on-top lines + osgText labels) plus the `math::BoundingBox` binding.
The Python `camera=None`, `frame_body`, `frame_region`, and adaptive-view paths
now bind and call those core bounds/framing helpers; Python retains only focus
name resolution and capture/search orchestration.
