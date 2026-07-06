# Agent Simulation Verification: Research and Design Record

Durable rationale for DART's agent-facing 3D-scene and physics verification
tooling. The how-to lives in
[`../onboarding/agent-sim-verification.md`](../onboarding/agent-sim-verification.md)
and the `dart-verify-sim` skill; this doc records _why_ the tooling is shaped
the way it is, backed by measured evidence.

## Problem

DART's domains — articulated-body dynamics, collision, robotics — require 3D
understanding that language models lack natively. An agent needs to inspect a
scene, run a simulation, and decide whether the result is correct, headlessly
and without hidden context. There is no established prior art for this exact
task: the literature splits into VLM physical-reasoning benchmarks (which show
models are weak at physics from images — e.g. PhysBench reports SOTA VLMs near
40% vs ~96% human) and engine self-testing (CPU/GPU parity, conservation as a
diagnostic). Neither is "an agent audits an engine's output," so DART's tooling
is built conservatively and self-grounding rather than mimicking an authority
that does not exist.

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
