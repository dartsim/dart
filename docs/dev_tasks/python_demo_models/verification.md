# Verification record

## Controller and terrain packet, 2026-10-13

Reference source: `4ff47d438`. Before-build native extension: `723389562`.
The intervening base changes repair node lifetime, image-error handling, SDF
parsing, hooks, CI, and program scope; they do not alter the four controller or
terrain implementations. The final comparisons use the rebuilt extension.

### Numerical evidence

- Biped and joint-constraint SPD controllers: 520 native/Python torque calls
  per controller across randomized states, four timesteps, and 200 real
  contact steps. Maximum absolute differences: 9.84e-13 and 9.09e-13.
  Active solver constraint forces and both biped ankle ranges were exercised.
- Operational-space control: 200 synthetic/KR5 cases, maximum force
  difference 1.59e-11. Separate 40-step synthetic and 200-step KR5 trajectories
  differed in state by at most 9.02e-17.
- Heightmap: five saved native cases, including default 100x100, nonsquare,
  single-column, and equal-height terrain. All 10,125 float draws, fields,
  scales, and next double draws matched exactly after rebuilding.
- Final scenes: seed 42, FCL, one simulation thread, checkpoints 0/20/200.
  Maximum state differences: biped 4.44e-15, joint constraints 4.43e-14,
  operational-space control zero. Heightmap has no articulated state; its
  exact field/scale/RNG oracle is recorded above.

The matrix operations use NumPy's solver instead of Eigen's inverse and permit
small floating-point differences in demo control. The sampled discrepancies
are substantially below the comparison tolerances (1e-9 absolute and 1e-8
relative for final scene trajectories); this is not a bit-identical controller
claim. C++ engine arithmetic and downstream simulation defaults are unchanged.
The comparison scripts, saved baselines, and result JSON are external transient
artifacts; transferred behavioral assertions live in the checked-in tests.

### Visual evidence

Matched before/after captures use seed 42, FCL, one thread, 50 steps, 960x600,
the scene's camera home, and Vulkan/Lavapipe. Both sides of all four pairs pass
`image-verdict`. Semantic inspection finds the same heightmap relief, standing
biped pose, joint-constraint biped pose, and KR5/target arrangement, with the
same scene controls and workspace. Timing/FPS readouts differ under concurrent
build load; these captures make no performance claim. Images and comparison
manifests remain outside Git.
The captures corroborate matched scene appearance and visible pose; they do not
measure contact geometry. The biped's existing camera crops the lower legs.
Debug overlays are unnecessary for this control-arithmetic relocation: the
force/state oracle supplies its behavioral evidence, and ordinary rendering
checks that the scene and controls remain visible. Contact geometry would need
a different camera and claim-specific debug layers.

### Checks and reviews

- `pixi run build-py-dev`: passed after all source deletions settled.
- `pixi run build`: passed.
- Guarded focused model/math/S3/S4/viewer tests: 153 passed.
- Independent heightmap helper gate on the rebuilt extension: 21 passed.
- Initial rebuild attempts exposed a stale removed-test property and a source
  deletion during compilation. Both were corrected by completing registration
  removal and regenerating after deletions; final builds passed.
- First independent review: clean after moving the keyboard-capture assertion
  into viewer tests, where it gates on viewer availability. Source equations,
  validation, lifetime, transferred native tests, binding dispatch, and package
  boundaries checked.
- Second independent review: clean after EOF cleanup and explicit capture
  scope. Reviewed native formula/test transfer, all four assessed comparison
  images, numerical scripts/results, callers, registrations, and docs.
- First aggregate run: all 191 CTests passed, then its launcher returned signal
  status 143 before the Python result. The full aggregate rerun passed; the
  interrupted command is not counted as a complete aggregate pass.
- `pixi run test-all`: 191 CTests and 1,642 Python tests passed; two
  Python-version-specific skips and 54 expected failures tracking pending
  DART 7 acceptance features.
- `pixi run test-gui`: 10 CTests and 539 Python tests passed; one
  Python-version-specific skip and two expected final-render-API failures.
- `pixi run cycle-demos`: all 37 scenes, twice, two frames, zero skipped.
- GUI-off (`DART_BUILD_GUI=OFF`, `DART_BUILD_DARTPY=OFF`): the complete `tests`
  target built; all eleven retained model-family CTests passed.
- Vanilla system ImGui: `tests` and dartpy built; all 71 GUI-free model/math
  Python tests passed. Import omits viewer, all five removed `_demos` exports
  are absent, and the catalog exits with its bundled-ImGui rebuild hint.
  Long build launchers were interrupted while Ninja continued; cached
  verification invocations confirmed completed builds with exit status zero.
- `pixi run check-ai-commands`: passed. Local-path check on changed and new
  files passed.
- `pixi run lint` and `pixi run check-lint`: passed, including semantic AI
  infrastructure checks. The final pre-commit lint covers the evidence-note
  updates. No behavior changes followed the completed gates.
- Final independent acceptance audit: recorded counts, skips, expected
  failures, completed variants, and catalog cycle match the logs. Only this
  four-family packet is complete; the remaining migration and WAM policy stay
  open.

## Arm and posture packet, 2026-10-14

Reference source and native demo implementations: `d13c20545`. A separately
archived extension retains those native helpers alongside the ordinary math
binding so the final oracle can compare both implementations in one process.
The ordinary optimizer lifetime fix does not alter the native posture objective.

### Numerical evidence

- RelaxedPosture: 600 cases at dimensions 0, 1, 3, 6, and 31, bounded and
  ideal modes, sparse weights and infinite bounds. Gradients match exactly;
  maximum cost difference is 1.14e-13. The original weighted residual is
  intentionally preserved rather than replaced with a squared-weight derivative.
- Atlas: 190 paired refreshes exercise all four dragged targets, held posture
  optimization, the balance control, and all eight movement keys. Positions,
  COM, objective cost, and end-effector matrices match exactly. Existing
  no-gravity support-polygon warnings prevent a claim of meaningful balance
  correction from this experiment.
- Arm helpers: four synthetic chains with sixteen ghosts each and 207 target
  control cases preserve geometry, float control rounding and the Eigen ZYX
  Euler branch. All nineteen catalog entries agree.
- Final real-arm oracle: all nineteen arms, at rest and sixteen posed ghosts,
  match native body/joint frames, every shape's local/world transform,
  dimensions, bounds, colors and collision-aspect absence exactly. The full
  prepared UR5 scene's 128 shapes, including target tools and marker, match
  exactly independent of pointer-based child ordering.
- Matched UR5 captures: camera and complete bounds agree exactly, as do all
  sixteen saved joint-position/body-frame arrays after activation.

The probes, baselines, captures and JSON results remain transient artifacts
outside Git. Checked-in tests retain the behavioral and ownership assertions.

### Visual evidence and diagnostic fix

The original 960x600 scene-home ssik captures cropped the chain and were rejected
as full-geometry evidence. Corrected matched 1600x1000 Vulkan/Lavapipe captures
fit the entire UR5 chain, base, eight ghost solutions, target axes and marker.
Both pass `image-verdict`; semantic inspection agrees with the structural oracle
and unchanged controls. The Atlas 960x600 pair corroborates visible upper-body
pose and controls, but crops feet and supplies no contact/balance evidence.
Ordinary arm rendering and existing target axes are sufficient corroboration
for this structural relocation; no additional core debug layers are needed.
These captures make no performance claim.

An apparent arm-bounds discrepancy was instead a bug in assessment traversal:
Python wrappers could die and their IDs be reused, causing child shapes to be
omitted. Retaining processed wrappers fixes it. The old implementation fails
the new complete InteractiveFrame inventory regression; all 68 assessment,
overlay and capture tests pass with the fix. It is a separate diagnostic commit.

### Checks and reviews

- The standalone optimizer fix has a failing retained-array oracle before the
  fix and six passing optimizer tests afterward. Arrays own independent storage,
  updates drive the native solver, and resized output is rejected. Two
  independent source reviews are clean.
- Review found that tiny quaternion directions could underflow normalization.
  Rejecting subnormal squared norms preserves the ordinary Eigen alignment
  path and closes the new source/target regression. Overflow, zero and
  nonfinite directions are also rejected.
- `pixi run build` and `build-py-dev`: passed; the reviewed math binding is rebuilt.
- Reviewed focused suite: 90 passed, including nine quaternion cases.
- Final sequential `pixi run test-all`: 189 CTests and 1,672 Python tests
  passed; two Python-version skips and 54 planned DART 7 acceptance failures.
- Final `pixi run test-gui`: ten CTests and 522 Python tests pass, with one
  Python-version skip and two planned final-render API failures.
- Strict catalog cycle: all 37 scenes twice, two frames each, zero skipped.
- GUI-off full `tests` target built. All ten retained native-model CTests pass
  across the main eight-test run and the additional adaptive-contact/arrow run.
- Vanilla system ImGui full `tests`/dartpy build passed; reviewed GUI-free
  model/math/optimizer suite has 191 passing tests. Catalog retains the intended
  bundled-ImGui rebuild hint.
- `pixi run lint`, `check-lint`, and the changed-file local-path check pass.
- Two independent arm/posture reviews are clean after the quaternion guard;
  both inspected numeric/visual evidence. The separate traversal review is clean.
- One aggregate attempt overlapped a separate extension rebuild; five spawned
  HTTPS-test workers saw a temporarily truncated shared object. That attempt
  is invalid. All fourteen affected cases pass after rebuilding; subsequent
  aggregate runs are sequential.

## Boxed-LCP and contact-replay packet, 2026-10-14

Reference native sources: `e18517de1`. Archived extensions retain the native
helpers before removal; the mixed LCP reference also has the ordinary Skeleton
wake setters and solver constructors. Comparison processes use a stable archive.

### Numerical and ownership evidence

- Boxed-LCP selection: 1,307 state comparisons plus seven matching exception
  pairs are bit-identical. These include 1,200 stacking steps, 104 sleep/wake
  snapshots, fixed/resting/awake/candidate bodies, island state, borrowed solver,
  collision detector/group/secondary backend identity, and SIMBICON harness
  preservation. The existing constraint solver is mutated rather than replaced.
- The real dwell regression settles first, wakes, accumulates partial quiet
  dwell, switches backend again and stays awake for fifty steps before sleeping
  on step fifty-one. Native comparison confirms this recipe; the initial fixture
  incorrectly overlooked the engine's equilibrium sleep credit and was corrected
  without weakening the post-switch assertions.
- LCP matched scene: all eighteen joint-position, velocity and body-transform
  arrays match after 800 direct physics steps, with seed 42, FCL, one thread,
  six skeletons, and PGS selected through the panel. Host steps are zero; this
  precomputed state capture makes no app-scheduling claim.
- Contact replay: 3,131 saved comparisons agree across all sixty spline knots,
  241 refreshes/history wrap, pause, derivatives, three regenerated depths,
  controls, contact forces, arrow geometry and histories. Physical-state, force,
  geometry and history arrays are bit-identical; 78 derived status arrays differ
  by at most 1.14e-13. Controls retain float32 rounding/clamps, four
  contacts retain their order, and exposed arrays/lists remain value copies.
- CID fitted scene: all ten position/velocity/acceleration, force, arrow and
  history arrays, the literal camera and complete focused bounds match exactly
  after sixty refreshes. The ordinary C++ inverse-dynamics solver remains intact.
- GUI-free tests retain validation, periodic interpolation/analytic derivatives,
  force-arrow hiding/colors, history order/wrap, ownership and cycle collection.
  Non-boxed validation uses a Python World subclass because Python cannot
  install a custom native ConstraintSolver; it exercises the helper's rejection
  without broadening the engine binding surface.

### Visual evidence

Matched Vulkan/Lavapipe captures fit the full five-box stack and, separately,
the complete squat pose, both feet and green force-arrow tips. Both original
pairs pass `image-verdict` and focused view assessment; semantic inspection
agrees with their complete state oracles and unchanged controls. One CID fit
passed geometric assessment but had feet obscured by the diagnostics dock;
that capture was rejected and both cameras were backed out before acceptance.
These captures corroborate one settled stack and kinematic pose/force-arrow
parity, not future stacking stability, physical ground contact or balance.
Ordinary rendering supplies the visible structural check; no contact/debug-layer
claim is made. No performance claim is made.

### Checks and reviews

- Post-removal `build-py-dev` and `build`: pass. Focused model/S3/S6a suite:
  56 passed against the rebuilt extension, after all native source deletions.
- `pixi run test-all`: 187 CTests and 1,694 Python tests pass, with two
  Python-version skips and 54 planned DART 7 acceptance failures.
- `pixi run test-gui`: ten CTests and 515 Python tests pass, with one
  Python-version skip and two planned final-render API failures.
- Strict catalog cycle: all 37 scenes twice, two frames each, zero skipped.
- GUI-off full `tests` target builds; all eight retained native-model CTests pass.
- Vanilla system ImGui full `tests`/dartpy build passes; all 220 GUI-free
  model/math/optimizer tests pass. The documented catalog rebuild hint remains.
- Two independent source/numeric/visual reviews are clean after qualifying the
  78 derived status vectors' tiny rounding difference. No implementation fix
  was required by review.
- `pixi run lint` and `check-lint` pass; final pre-commit lint and staged
  local-path check cover the completed evidence notes.

## Soft-worm and adaptive-contact packet, 2026-10-14

Reference native sources: `3e55e373c`. Immutable native and mixed extensions
retain both helpers; the mixed reference also contains the ordinary soft-body
bindings. Final deletion and shared registration follow the accepted comparisons.

### Numerical, boundary and ownership evidence

- Worm: all 3,001 states of 1,694 values match bit-for-bit, including all 130
  point masses' local positions, velocities, world positions and forces,
  joint positions/velocities/accelerations/forces/commands, body transforms and
  the ordered long-double checksum. Both the original native baseline and a
  separate Python 3,000-step repeat are identical. Root displacement is
  1.6374483483834361; final checksum is 91066.0472284822.
- Review found differences in numeric conversion, strict boolean validation
  and nonfinite phase handling. The corrected gait matches all 54 native input
  cases and 22 exception pairs, including numeric protocols, NumPy scalars,
  nonfinite/overflow phases and disabled calls. The complete trajectory and
  independent repeat were rerun after the correction and remain bit-identical.
- Adaptive contact: 37 saved trace arrays agree over 2,000 steps in adaptive
  and all-active worlds. All point/joint state, transforms, contacts and flags
  are exact; checksum/surface diagnostics differ by at most 5.56e-17. Repeated
  Python runs are exact. Each body has 86 points; adaptive activation is reduced
  for 1,987 steps, both worlds have contacts for 1,596 steps, final activation
  is 37/86, and maximum surface-pose delta is 0.0595341, below the 0.25 gate.
- The second review found matching numeric-conversion drift in the pusher.
  The corrected conversion and fourteen regression cases pass; all 54 native
  boundary cases and twenty exception pairs match with bit-identical poses,
  including buffers, float subclasses, numeric protocols and nonfinite phases.
  The full trajectory/repeat recheck after this correction preserves all 37
  arrays within the same diagnostic-roundoff bounds and repeats exactly.
- Typed settings, pusher nonfinite behavior, finite guards, copied outputs and
  child resource lifetime are preserved. Borrowed children retain the native
  Skeleton; the unused private Python Model shell can release. No artificial
  Python shell pins are added. Running scenes retain their World/model.
- Ordinary bindings expose copied read-only point arrays, checked point lookup,
  adaptive settings and the existing revolute soft-body factory. Focused tests
  cover invalid indices, array independence, direct nonfinite native state and
  retained point/body resource use. The still-native history tests now use an
  independent generic soft-body C++ fixture and the Python worm fixture.
- Second review reproduced a stale point handle after inherited `BodyNode.copy`
  shrinks soft topology. Retaining the body cannot retain points it deletes;
  both properties and the existing point-impulse API need destruction-aware
  input rejection. A per-wrapper native destruction observer now rejects
  expired PointMass conversion and skips expired identities at reused native
  addresses. All three subprocess regressions fail against the old extension,
  including a regrowth SIGSEGV. Live identity, equal-size copying, movement,
  getters/setters, impulse arguments and cleanup have regression coverage;
  all three regressions pass after rebuilding, within the 66-test focused gate.

### Visual evidence

Six matched Vulkan/Lavapipe worm captures at steps 0, 1,500 and 3,000 fit all
five colored links; view assessments and image verdicts pass. All 3,001 rows of
the scene's 134-value trace and construction/material metadata agree exactly.
Images corroborate complete geometry, gait and root advancement; the captured
UI contact count is zero, so these images do not prove floor contact.

Four fitted adaptive/all-active captures after 2,000 steps fit the entire
ellipsoid and pusher. All nineteen state/marker arrays per mode, literal cameras
and bounds match exactly; image verdicts pass with zero differing pixels.
Semantic inspection confirms the adaptive lower/side gold region and 37/86
count, compared with all-active gold surface and 86/86. Physical-body view
assessments pass. Raw marker reports retain their ambiguity flag because the
embedded glyphs overlap; inspection assesses the displayed region, not each
glyph's separation. Markers remain a contact-nearest region estimate sized to
the public active count, not exact native activation membership. Ordinary
rendering corroborates these structural claims; no performance claim is made.

### Checks and reviews

- Final mixed-reference focused gates: 37 worm tests and 53 adaptive tests pass.
- Post-removal worm gate: 37 passed; final core/ownership/history gate: 66 passed.
- Independent C++ WorldHistory fixture builds and its CTest passes.
- Post-removal `build-py-dev` and `build` pass. GUI-off full `tests` build
  and all six retained native-model CTests pass. Vanilla system ImGui full
  `tests`/dartpy build, final lifetime rebuild and 341 GUI-free
  model/math/optimizer/core tests pass. Catalog retains the rebuild hint.
- Two independent final source/numeric/visual reviews are clean after both
  conversion fixes and the lifetime guard; all ten original captures were
  inspected.
- Final sequential `pixi run test-all`: 185 CTests and 1,782 Python tests pass,
  with two Python-version skips and 54 planned DART 7 acceptance failures.
- Final `pixi run test-gui`: ten CTests and 516 Python tests pass, with one
  Python-version skip and two planned final-render API failures.
- Strict catalog cycle: all 37 scenes twice, two frames each, zero skipped.
- An earlier aggregate launcher returned status 143 after printing both
  passing subgates; it is excluded as an aggregate result. The final post-fix
  launch completes with status zero and the counts above.
- `pixi run lint`, `check-lint`, diff and local-path checks cover the final
  source; final pre-commit lint also covers the completed evidence notes.

## Limits

No WAM backend change is implemented. The analytical prototype's sampled
accuracy does not establish full reachability, exact IKFast branches,
interactive continuity, or performance. Windows native runtime validation is
unavailable on this Linux host. The remaining native families and `_demos`
module are intentionally present until their ports and decisions are complete.
