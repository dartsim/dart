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

## Limits

No WAM backend change is implemented. The analytical prototype's sampled
accuracy does not establish full reachability, exact IKFast branches,
interactive continuity, or performance. Windows native runtime validation is
unavailable on this Linux host. The remaining native families and `_demos`
module are intentionally present until their ports and decisions are complete.
