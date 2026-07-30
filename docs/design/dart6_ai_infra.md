# DART 6 AI Infrastructure Bucket Decision

## Decision

DART 6.20 adopts the AI-infrastructure documentation buckets that solve
release-branch maintenance problems now:

- `docs/plans/` for living priority, roadmap, and gate state.
- `docs/design/` for durable release-branch design rationale.
- `docs/background/` for theory, paper, and reference foundations.
- `docs/assets/` for durable repository documentation assets.

This is a DART 6 adaptation, not a wholesale import of DART 7 project state.
The release branch remains a compatibility lane for the established DART 6 API,
installed headers, package components, and Gazebo/gz-physics consumers.

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

## DART 6 Differences From DART 7

DART 7 `main` can use clean API-breaking changes to simplify the future
simulation stack. DART 6.20 cannot use that latitude as proof that a
release-branch change is safe.

For DART 6:

- DART 7 docs and code are reference evidence only.
- Public headers, exported package components, and gz-physics/gz-sim behavior
  remain compatibility constraints.
- GPU, clean-break API shape, EnTT world plumbing, and DART 7-only package
  restructures stay out of release-branch plans unless a maintainer explicitly
  accepts a scoped backport.
- Durable DART 6 decisions should say which compatibility surface they protect
  and which gate proves it.

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
DART 7 renderer or binding internals are comparison evidence, not backport
requirements.

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

The DART 7 lineage in
[#3313](https://github.com/dartsim/dart/pull/3313),
[#3320](https://github.com/dartsim/dart/pull/3320),
[#3371](https://github.com/dartsim/dart/pull/3371), and
[#3386](https://github.com/dartsim/dart/pull/3386) remains comparison evidence.
The release adapts portable text/image handoff and semantic-review contracts,
including a hashed verification bundle and fail-closed publication. It omits
Filament, nanobind, DART 7 renderer descriptors, and main-only camera/viewer
types. This apply/adapt/omit record prevents a later model upgrade from
mistaking intentional branch differences for drift.

### Post-Merge Parity Audit For #3403

The final DART 7 comparison point is
[#3403](https://github.com/dartsim/dart/pull/3403), merged as
`83110ef54abf41f54c1e03500e49c1c12c305b8a`. Its tree is identical to the
reviewed head `b507b288e8de267f0a3518e5bde2efb948337291`; no squash or merge drift
changed the audited implementation. The DART 6 comparison was then refreshed
against `release-6.20` at `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`
([#3381](https://github.com/dartsim/dart/pull/3381)), after the DART-owned
collision engine was consolidated into `DARTCollisionDetector`.

The resulting branch verdict is:

- **Apply:** keep the model-agnostic upgrade workflow, documentation as
  operational memory, absence of repository model pins, text/image semantic
  review contract, hashed verification bundle, and fail-closed evidence
  publication. Publication revalidates every selected artifact's size and
  SHA-256 digest plus claim coverage and pass state before any GitHub call,
  then uses content-addressed assets and records path/size/digest/URL bindings
  without cross-content clobber.
- **Adapt:** implement the DART 7 Filament/nanobind/viewer intent through the
  established DART 6 C++17, pybind11, OSG `OffscreenViewer`,
  `agent_capture.py`, `agent_view_quality.py`, and core `DebugOverlay` path.
  The visual fixtures and raycast fallback use the consolidated
  `DARTCollisionDetector`; the release-only owned contact snapshots and
  same-camera overlay checks strengthen that adapted path, and semantic image
  inspection pins the multi-shape smoke to an orthogonal camera where its
  labels and contact markers remain readable. None depends on the removed
  temporary `NativeCollisionDetector` name; they are not missing main APIs.
- **Omit:** do not import DART 7-only viewer C++ APIs, nanobind
  descriptor/stub machinery, trajectory/scene-dump tools, C++23 surfaces, or
  clean-break simulation architecture. Those surfaces do not exist on the
  compatibility branch and are not prerequisites for its visual evidence
  contract.

Future model audits must repeat this comparison against the then-current final
trees. They should improve the shared workflow or capability contract when a
new model exposes a portable weakness, rather than adding a model-named command
or treating an intentional renderer/API difference as drift.

## Intentional Non-Adoption

`docs/python_api/` from DART 7 is not adopted in this AI-infra change. On this
release branch, dartpy user-facing documentation remains under
`docs/readthedocs/dartpy/`, and C++ API generation input remains under
`docs/doxygen/`. Moving DART 6 to the DART 7 reusable Python autodoc layout
would be a separate documentation-build change, not an AI-infra bucket
requirement.
