# PR-3a — soft-foot SIMBICON locomotion (Jain/Liu "biped push recovery")

Kickoff spec for the first Jain/Liu parity build (plan `10-full-parity-execution-plan.md`, Part A). Goal:
reproduce the paper's soft-vs-rigid contact locomotion claim — *"soft contact
withstands larger perturbations while maintaining more ground contact points"* —
as a GUI-free model + `dart-demos` scene + model test, ABI-safe additive.

## Verified reuse facts (2026-07-23)

- The SIMBICON controller exists and is **GUI-free**:
  `examples/demos/scenes/atlas_simbicon/{Controller,State,StateMachine,TerminalCondition}.{hpp,cpp}`
  (only `AtlasSimbiconScene.cpp` pulls in GUI/OSG).
- Controller ctor: `Controller(SkeletonPtr atlas, ConstraintSolver* solver)`;
  `update()` each step; finds `pelvis`, `l_foot`, `r_foot`, and DOFs
  `l_leg_hpx/hpy`, `r_leg_hpx/hpy` by name. Its `BodyContactCondition` reads
  `BodyNode::isColliding()` — works for rigid or SoftBodyNode feet.
- Scene setup recipe (from `AtlasSimbiconScene.cpp:167-205`): `World::create()`;
  `urdf.parseSkeleton("dart://sample/sdf/atlas/ground.urdf")`;
  `SdfParser::readSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.sdf")`;
  add ground then atlas; `atlas->setPosition(0, -0.5*pi)`;
  `world->setGravity({0,-9.81,0})` (**Y-up — the controller math hardcodes Y as
  vertical**); pelvis push via `pelvis->addExtForce(...)` in preStep.
- Soft variant: swap the SDF to
  `dart://sample/sdf/atlas/atlas_v3_no_head_soft_feet.sdf` (SoftBodyNode feet,
  same `l_foot`/`r_foot` names, kv=50000/ke=100). Everything else identical.

## Deliverables (3-part pattern, copy `SoftWorm*`)

1. `examples/demos/scenes/soft_foot_simbicon/SoftFootSimbiconModel.{hpp,cpp}`
   (GUI-free): `enum class Feet {Rigid, Soft}`; `Model createModel(Feet)`;
   `applyPush(Model&, Vector3d, int steps)`; `step(Model&)` (push countdown +
   `controller->update()` + `world->step()`); metrics `pelvisHeightY`,
   `isUpright` (pelvis Y above fall threshold), `footContactCount` (contacts on
   l_foot/r_foot), `isFinite`, `stateVector` (full skeleton + point-mass
   state, for exact determinism comparison), `resetModel` (skeleton config,
   soft point masses, gait phase, pending push); helper
   `maxRecoverablePush(Feet)` (sweep push magnitudes, return largest from which
   the biped stays upright N steps).
2. `examples/demos/scenes/soft_foot_simbicon/SoftFootSimbiconScene.cpp` →
   `makeSoftFootSimbiconScene()`; register in `examples/demos/Registry.cpp`
   (Soft Bodies group) + declare in `examples/demos/scenes/Scenes.hpp`. Feet-type
   toggle, push keys, panel: contact count / pelvis height / upright / gait.
3. `tests/integration/test_SoftFootSimbiconModel.cpp` (GUI-free), gtest suite
   `SoftFootSimbiconModelTest`.

## CMake wiring

- Demos: add `scenes/soft_foot_simbicon/*.cpp` to the subdir globs in
  `examples/demos/CMakeLists.txt` (alongside the `atlas_simbicon` glob at ~:150).
- Test: in `tests/integration/CMakeLists.txt`, compile into the existing
  `test_ConstraintSolver` target (like `SoftWormModel`): the model `.cpp` **and
  the four controller sources** `Controller.cpp/State.cpp/StateMachine.cpp/
  TerminalCondition.cpp` — but NOT `AtlasSimbiconScene.cpp` (GUI). Add
  `add_test(NAME test_SoftFootSimbiconModel COMMAND $<TARGET_FILE:test_ConstraintSolver>
  --gtest_filter=SoftFootSimbiconModelTest.*)` with the `0 tests` FAIL regex.

## Acceptance gates (paper row)

- Soft-foot biped stands finite + upright for ≥ N steps (no fall) under the
  controller — deterministic (two runs produce an exactly equal `stateVector`).
- Reset is bit-for-bit equivalent to a freshly built model: a reset model and a
  fresh model stepped in lockstep for 300 steps stay exactly equal, in both the
  physical state and the gait phase. That requires restoring the soft feet's
  point-mass state, the gait phase, the contact bookkeeping, the world clock and
  resting snapshots, and rebuilding the collision detector's incremental
  broadphase.
- **Push recovery: measured, currently NOT reproducing.** Once the two models
  are made comparable, `maxRecoverablePush` gives soft 4000 N against rigid
  8000 N, so the paper's "withstands larger perturbations" does not hold with
  the shipped soft-feet asset. The gate records both thresholds and asserts only
  that neither saturated the sweep. A stiffness sweep shows the cause is the
  asset's compliance, not the contact model: at kv 1e6 / damp 5e3 the soft feet
  recover 12000 N and do beat rigid, against the shipped kv 5e4 / damp 1e3.
  **Open decision for a maintainer**: retune the shared asset, override the
  stiffness scene-locally, or accept the negative result. Until then this row is
  carried by the contact-count claim only.
- **Comparable models**: the rigid and soft bipeds must weigh the same and
  present the same number of foot collision surfaces, normalized on the loaded
  instance rather than in the shared `dart://sample` asset. A `<soft_shape>`'s
  `<total_mass>` is added on top of its link mass, and its generated
  `SoftMeshShape` is collidable in addition to the link's rigid STL collision
  mesh; without correcting both, the gates measure ground load and duplicated
  geometry rather than deformability.
- **Contact count: soft maintains ≥ rigid** foot contacts over a settled window
  (paper: soft maintains more contact points).
- Finite-state throughout; GUI-free numerical oracle (no visual claim required
  for the model test).

## Constraints

- ABI-safe additive; touch only new files + `Registry.cpp`/`Scenes.hpp`/the two
  `CMakeLists.txt` (no existing public headers/classes).
- Build with `DART_DISABLE_COMPILER_CACHE=ON pixi run cmake --build
  build/default/cpp/Release --target test_ConstraintSolver` and run the filtered
  test; `pixi run lint` before commit. Release-branch quality.
- New GUI examples belong in `dart-demos`, not standalone executables.
