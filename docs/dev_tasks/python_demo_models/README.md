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

## Inventory and remaining packets

The initial inventory found 15 binding families, 39 native source/header files
(22,020 lines, including 14,933 generated WAM solver lines), 16 binding files,
15 scene callers, registry gates, Python/native tests, and four build surfaces.

After the arm/posture packet, nine native binding families remain:
boxed-LCP selection with missing engine wake bindings; contact inverse dynamics;
soft worm; adaptive soft contact; SIMBICON and
soft-foot SIMBICON; point-cloud sampling with mesh API gaps; world history with
missing snapshot types; WAM; and final module/build removal.

`ContactArrowLayout` is generic GUI code used by `dart/gui/app`; move it to the
GUI owner before removing the model library. The private native app host stays
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
