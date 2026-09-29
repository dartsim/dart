# DART 6 AI Infrastructure Bucket Decision

## Decision

DART 6.20 adopts the AI-infrastructure documentation buckets that solve
release-branch maintenance problems now:

- `docs/plans/` for living priority, roadmap, and gate state.
- `docs/design/` for durable release-branch design rationale.
- `docs/background/` for theory, paper, and reference foundations.
- `docs/assets/` for durable repository documentation assets.

The release branch remains a compatibility lane for the established DART 6 API,
installed headers, package components, and Gazebo/gz-physics consumers:

- Public headers, exported package components, and gz-physics/gz-sim behavior
  remain compatibility constraints.
- GPU work and API-breaking changes stay out of release-branch plans unless a
  maintainer explicitly accepts them.
- Durable DART 6 decisions should say which compatibility surface they protect
  and which gate proves it.

## Rationale

Before these buckets, long-running DART 6 work had to keep roadmap dashboards,
design decisions, paper matrices, and reusable documentation evidence inside
`docs/dev_tasks/`. That made task retirement harder because durable facts had
no precise owner after the temporary folder was removed.

The new buckets separate lifecycle from topic:

- `docs/plans/` owns mutable operating state.
- `docs/design/` owns durable engineering decisions.
- `docs/background/` owns reusable theory and reference context.
- `docs/assets/` owns durable doc media.
- `docs/dev_tasks/` stays temporary and branch/session-local.

## Evolving Models Without Catalog Drift

Model and coding-agent upgrades use one maintained `dart-model-upgrade`
workflow rather than a new command per model family. Its intake, control
capture, classification, comparison, verification, and closeout procedure is
model-agnostic. A bounded target-specific routing example may be replaced when
official guidance changes; it is not a permanent taxonomy.

The audit boundary includes tracked documentation because `docs/` carries both
in-session context and across-session project state. The durable owners are
`docs/ai/`, `docs/plans/`, `docs/dev_tasks/`, and the handbook, design,
release, or module references routed into a task. The audit checks discovery,
freshness, duplication, context cost, resume quality, and usefulness to human
maintainers as well as agent behavior.

The same model-independent rule applies to verification harnesses: a gate must
prove that its test bodies ran, not merely that a test process exited zero.
Canonical CTest and pytest routes therefore clear whole ambient control
families, use branch-owned configuration and plugin boundaries, reject
successful zero-body execution, and exercise passing and failing probes through
the checker.

## Visual Verification North Star

DART 6 visual verification follows one evidence chain:

1. a text oracle establishes scene, dynamics, collision/contact, or constraint
   correctness;
2. core bounds and collision raycasts select and assess a claim-tied camera;
3. the OSG offscreen renderer captures the scene with only the necessary
   `DebugOverlay` layers;
4. machine pixel checks establish artifact integrity or reference difference;
5. an image-capable reviewer inspects the selected still or temporal frames and
   records visible observations separately from the text result;
6. publication reconciles both channels, names a pass/fail/uncertain verdict,
   and states what the evidence does not prove.

Images corroborate; the text oracle decides correctness. A passing view report
or pixel verdict is not semantic inspection, and text/image disagreement
cannot be averaged into a pass. The capture sidecar identifies deterministic
static or start/middle/end inspection targets so future image-capable models can
exercise the same contract without prompt-specific frame selection.

The DART 6 implementation stays on its existing C++17, pybind11, OSG
`OffscreenViewer`, core `DebugOverlay`, and release camera-assessment path.

### Capability Lineage And Release Verdicts

The release workflow is the cumulative result of these merged changes:

- [#3304](https://github.com/dartsim/dart/pull/3304) established usable
  translucent, dynamic soft-body visualization. Preserve the OSG rendering
  behavior; its older standalone capture entrypoint has since converged into
  `dart-demos`.
- [#3314](https://github.com/dartsim/dart/pull/3314) added the GLX-pbuffer
  `OffscreenViewer`, default camera, dartpy bindings, and initial
  verdict/golden/sheet tools. Preserve the C++17/pybind11 API and adapt its
  agent harness around viewport-aware framing and explicit missing-bounds
  failures.
- [#3374](https://github.com/dartsim/dart/pull/3374) added assessed viewpoints,
  ten OSG `DebugOverlay` layers, capture sidecars, and claim-tied
  selection/publication. Preserve the core OSG path and improve the evidence
  contract rather than adding image-space annotations.
- [#3385](https://github.com/dartsim/dart/pull/3385) made claim-specific World
  factories and engine-rendered overlay checks non-skippable under Xvfb.
  Preserve the same-camera A/B and per-layer pixel gates.

The branch keeps the model-agnostic upgrade workflow, documentation as
operational memory, absence of repository model pins, text/image semantic
review contract, hashed verification bundle, and fail-closed evidence
publication. Publication revalidates every selected artifact's size and
SHA-256 digest plus claim coverage and pass state before any GitHub call, then
uses content-addressed assets and records path/size/digest/URL bindings without
cross-content clobber.

The visual evidence contract runs on the established DART 6 C++17, pybind11,
OSG `OffscreenViewer`, `agent_capture.py`, `agent_view_quality.py`, and core
`DebugOverlay` path. The visual fixtures and raycast fallback use the
consolidated `DARTCollisionDetector`; the owned contact snapshots and
same-camera overlay checks strengthen that path, and semantic image inspection
pins the multi-shape smoke to an orthogonal camera where its labels and contact
markers remain readable. None depends on the removed temporary
`NativeCollisionDetector` name.

Future model audits should improve the shared workflow or capability contract
when a new model exposes a portable weakness, rather than adding a model-named
command.
