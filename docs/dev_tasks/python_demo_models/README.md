# Python demo implementation

## Intent and acceptance

The maintainer requested C++ engine, IO, and GUI libraries with Python bindings
for those libraries; demo implementation belongs in Python. Remove
`dartpy._demos` and `dart-demo-models` as the ports finish. Preserve installed
headers, exported package components, C++17, and Gazebo behavior.

The catalog retains its scenes, controls, and simulation behavior. Each port
must replace its callers, preserve validation and ownership, transfer native
behavioral checks into Python, and remove its native implementation and binding.
Numerical comparisons against the current native implementation precede removal;
scene construction/stepping tests and assessed captures corroborate them.

This work is separate from `dart7_demo_followups`: its marker fix remains on
`fix/demo-consolidation-followups`, pending publication approval and hosted CI.

## Completed packet: Python controllers and terrain

- Branch: `refactor/python-demo-controllers`, based on fetched `origin/main`
  `4ff47d438` (2026-10-13).
- Target and milestone: `main`, open `DART 7.0.0`.
- Value: remove four native demo implementation families using engine bindings
  that already exist, without waiting on the WAM solver decision.
- Scope: heightmap generation; biped-standing, joint-constraint, and
  operational-space controllers; their scene/test callers; private build
  registration. Keyboard capture is GUI functionality and moves to the viewer
  binding.
- Non-goals: changing core physics, installed APIs, numerical algorithms, the
  WAM backend, world-history semantics, and the other demo families.
- Gates: `pixi run build`, `pixi run build-py-dev`, focused Python and native
  tests, `pixi run test-all`, `pixi run test-gui`, strict catalog cycling,
  numerical native/Python comparisons, matched assessed captures, and lint.
  Private-library GUI-off/vanilla-ImGui builds check the documented boundary.
- Compatibility: private model source/bindings change; installed headers,
  package exports, ABI-sensitive engine classes, and default physics do not.
- Review: two clean independent passes required on the final packet.
- Status: locally complete and verified; no publication authorization. The
  overall `_demos` removal remained open for eleven families at this point and the generic
  GUI contact-arrow helper.

The four Python implementations and transferred tests replace their native
source, bindings, and C++ test registration. Engine float random draws are
exposed through `math.Random.uniform_float`; keyboard focus is exposed through
the viewer. No installed C++ interface or engine physics implementation changes.
See [verification](verification.md) for numerical, visual, build-variant, test,
and independent review evidence.

## Arm helpers and posture objective

- Branch: `refactor/python-demo-arm-posture`, a local child of
  `refactor/python-demo-controllers` at `d13c20545`.
- Dependency: the completed four-family packet. The fetched main is
  `69526d6ae`; its newer delta changes AI workflow files/tests only.
- Value: remove catalog data, arm construction/visualization, target controls,
  and the Atlas posture objective from the extension.
- Scope: Python model helpers, Atlas/ssik scene callers and registration,
  matching Python/native tests, private CMake/module registration, and a
  regular quaternion vector-alignment binding that reuses the engine's
  cylinder-orientation routine.
- Assumptions: the objective preserves the original weighted residual gradient;
  generic optimizer callbacks must receive a valid gradient through ordinary
  bindings. Probe the callback before relying on it and fix a genuine binding
  gap in its library owner if necessary.
- Non-goals: WAM policy, other model families, unrelated marker behavior,
  installed C++ interfaces, engine physics, and publication.
- Acceptance: numerical native/Python comparison before native removal,
  real WholeBodyIK objective callbacks/ownership, all nineteen arm FK cases,
  ghost geometry/lifecycle and control parity, assessed matched captures,
  transferred GUI-free model tests, build and relevant full/GUI/variant gates,
  lint, and two independent clean reviews.
- Status: locally complete and verified; no publication authorization. The
  core callback bug and assessment traversal fix have separate commits.

### Optimizer callback bug fix

The gradient callback writes through correctly, but its NumPy array borrows
native scratch storage and may escape the callback. A live probe confirmed
retained gradients alias the next iteration's zeroed buffer. Input arrays are
independently copied and need no change. A focused fix passes an owned gradient
copy, copies output back, and rejects changed size; it is isolated on
`fix/python-optimizer-gradient-lifetime`, with convergence/retention regression
tests. The port branch inherits that local fix at `0df4896ab`.

Changelog decision: entry required for this standalone public Python binding
bug fix, rather than deferred program feature work. Target DART 7.0.0 / Python:
keep optimizer callback gradient arrays alive after callbacks while applying
their updates to the solver. The entry is recorded without a publication link.

### Capture bounds bug fix

The native/Python geometry oracle found no arm mismatch. A bounds discrepancy
exposed recycled Python wrapper IDs in the assessment traversal: after a
wrapper died, its ID could match another child and omit that shape. Retaining
visited wrappers fixes the traversal, with a full InteractiveFrame inventory
regression and separate commit `6a04cf3ec`. This internal diagnostic fix needs
no release entry. Corrected matched captures and all nineteen real-arm geometry
comparisons corroborate the port.

## Changelog decision

- Mode: decide.
- Base evidence: fetched `main` at `4ff47d438`, DART 7.0.0 release section,
  and the Python-first program's hot-file rule.
- Scope evidence: four private model/binding ports and two regular library
  binding additions (`Random.uniform_float`, viewer keyboard capture).
- Decision: entry deferred to M6b's milestone consolidation PR, as required
  for program feature work; this is not a standalone bug fix.
- Target: DART 7.0.0 / Python.
- Entry text: recorded in `dart7_python_first/changelog-drafts.md`.
- Reason: the library binding additions are user-visible; the private source
  catalog model relocation has no separate release migration contract.
- Follow-up: consolidate the drafts at M6b exit and add the publication link.

## Boxed-LCP selection and contact inverse dynamics

- Branch: `refactor/python-demo-lcp-contact`, local child of arm/posture
  commit `e18517de1`.
- Value and scope: port backend selection/wake handling and the Atlas squat
  replay, keyframes, controls, histories and force visuals to Python. The
  ordinary `dynamics.ContactInverseDynamics` solver stays in C++; bind existing
  Skeleton sleep-candidacy/island setters to preserve the native wake recipe.
- Compatibility: preserve borrowed boxed-solver identity, sleeping/dwell and
  fixed-body behavior, four-contact order, float controls, native value copies,
  replay derivatives, history-ring order and visual ownership. No C++ engine
  headers, implementations, package exports or physics defaults change.
- Non-goals: WAM policy, other families, unrelated marker repair and publication.
- Acceptance: direct native/Python comparisons before native removal, real
  settling/stacking and complete squat-replay oracles, transferred GUI-free
  tests and ownership checks, fitted matched assessed captures, default/GUI
  and build-variant gates, catalog cycle, lint and two clean reviews.
- Status: locally complete and verified. Native helpers/bindings/tests and
  shared registration are removed after direct parity. No publication authorization.
- Changelog: program feature work defers to the M6b consolidation draft; ordinary
  binding additions receive a draft entry. No standalone bug fix is proposed.

## Soft worm and adaptive soft contact

- Branch: `refactor/python-demo-soft-models`, local child of the completed
  boxed-LCP/contact packet at `3e55e373c`.
- Value: move soft-body construction, gait/pusher control, diagnostics and
  estimated activity-marker ranking into Python. Actual soft dynamics and
  adaptive activation remain ordinary engine implementations.
- Scope: both models/scenes/tests; PointMass state access, checked point lookup,
  adaptive SoftBodyNode settings and revolute soft-body factory bindings;
  independent soft-body fixture for the still-native WorldHistory test.
- Compatibility: preserve mass/stiffness/damping, topology and names, gait and
  pusher arithmetic, ordered long-double checksums, marker estimates and copies.
  No engine headers/implementations, package exports or physics defaults change.
- Ownership: borrowed children retain their native Skeleton resource. The unused
  private Python Model shell can release; do not add pins solely to preserve its
  weakref lifetime. Running scenes retain their World/model as before.
- Acceptance: complete native/Python point-state comparisons and transferred
  3,000-step worm displacement/exact-repeat gate; 2,000-step adaptive/all-active
  comparison with 86 points, contact activity, reduction for over half the
  steps, final activation at most 75%, surface delta at most 0.25 and exact
  repeat. Preserve typed/nonfinite input behavior, ownership and finite guards.
  Matched fitted timeline/activation captures corroborate structural claims;
  marker glyphs remain an approximate region estimate. Relevant full/GUI/variant
  gates, cycle, lint and two independent clean reviews finish the packet.
- Status: locally complete and verified. Native helpers/bindings/tests and shared
  registration are removed after parity. Both conversion fixes and a point
  handle destruction guard pass their regression and broad gates; two
  independent final reviews are clean. No publication authorization.
- Changelog: defer ordinary engine binding features to the M6b consolidation
  draft. The model relocation has no separate public release contract.

## Current packet: world history and point-cloud sampling

- Branch: `refactor/python-demo-state-mesh`, local child of the completed soft
  packet at `5a3a36635`, with isolated conversion follow-up `3c9ba6223`.
- Value: move history recording/playback and seeded sensor sampling, orbit,
  colors and voxel updates into Python using ordinary library bindings.
- Scope: both models/scenes/tests and native removal; owning configuration and
  composite snapshots, direct body state access, point acceleration copies,
  copied grouped mesh vertices, integer random draws, fourth point-render mode
  and shared voxel-update preflight validation.
- Compatibility: preserve history cap/branching/counters, positional matching,
  state copies and world-time omission. Preserve mesh-group sampling, random
  draw order, float narrowing, fallback, color quirks and mutation order.
  Reject mismatched soft-point snapshot counts in body and skeleton restores
  before native restoration can invalidate point indices. No engine headers,
  implementations or exports change.
- Acceptance: native/Python full snapshot and seeded sampling comparisons,
  input boundary/mutation/copy/ownership checks, transferred native coverage,
  direct point acceleration restore checks, matched assessed playback/sensor
  captures, full/GUI/variant gates, cycle, lint and two independent reviews.
- Status: locally complete and verified. Native helpers/bindings/tests and
  registration are removed after complete paired comparisons. Snapshot safety,
  full/GUI/variant gates, catalog cycle and two independent final reviews pass.
  No publication authorization. Feature changelog additions defer to M6b.

## Inventory and remaining packets

The initial inventory found 15 binding families, 39 native source/header files
(22,020 lines, including 14,933 generated WAM solver lines), 16 binding files,
15 scene callers, registry gates, Python/native tests, and four build surfaces.

After the state/mesh deletions, three native binding families remain: SIMBICON,
soft-foot SIMBICON and WAM, followed by final module/build removal. World history
uses existing owning snapshots through ordinary library bindings. Point-cloud
sampling uses copied per-submesh vertices, integer engine random draws and
shared voxel preflight validation. Both preserve their demo algorithms.

The bounded SIMBICON audit found no further design decision: Python can own
both controllers/models and their state machines. Bind existing point mass,
collision flags, reset operations, copied soft point/face data and owning
TriMesh construction as needed. Preserve Y-up, strict timer transitions,
machine-state retention, the legacy foot-harness behavior, link mass versus
total soft mass, copied inertia updates and complete reset ordering. Transfer
the deterministic gait/reset, settled contact and full recovery-sweep gates;
remeasure recovery thresholds before retaining panel claims. Borrowed bodies
retain their Skeleton resource; controllers retain their World and remove
pelvis harnesses on destruction, including cyclic collection.

`ContactArrowLayout` is generic GUI code used by `dart/gui/app`; move it to the
GUI owner before removing the model library. Its private app source can be
compiled directly into the existing core test binary to retain GUI-off checks;
no new library or installed header is needed. The private native app host stays
with the GUI implementation. Discuss consequential gray areas one at a time.

## Decision needed: WAM analytical IK

The existing generated IKFast solver fixes `/j3` as its free joint. A Python
prototype using existing optional ssik 8.1.0 and closed-form `seven_r.srs`
solved 40/40 valid sampled WAM configurations without refinement or rescue.
Input FK matched DART within 8.66e-15; independently applied solutions matched
the end-effector transform within 6.95e-9. `/j3` changed in 39/40 cases by up to
1.570795 radians. Exact branch selection, interactive continuity, wider
reachability, and performance are unverified.

Recommended: accept different joint selection and replace/relabel the WAM demo
as Python analytical IK. Alternative: preserve exact IKFast behavior with a
generic engine binding and an optional compiled robot-specific solver artifact.
The maintainer has not selected an option. Continue independent ports only.

## Completion

The project completes after all families are ported, gray decisions are
resolved, `_demos` and private demo-native code/build dependencies are removed,
tests and docs reflect the accepted boundary, and required evidence/reviews
pass. Promote durable facts to architecture, scene rules, binding and testing
owners before retiring this folder. GitHub publication requires explicit
approval.
