# AVBD Paper / Reference Gap Audit

This audit grounds PLAN-104 against _Augmented Vertex Block Descent_ and the
public 2D/3D reference demo repositories. It records the full implementation
target for DART's AVBD work. It is a planning artifact; it does not itself
claim implemented behavior.

## Sources

- Chris Giles, Elie Diaz, and Cem Yuksel. "Augmented Vertex Block Descent."
  _ACM Transactions on Graphics_ 44(4), Article 90, 2025.
  DOI: <https://doi.org/10.1145/3731195>.
  Project page:
  <https://graphics.cs.utah.edu/research/projects/avbd/>.
  Paper PDF:
  <https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf>.
- Online demos:
  <https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo2d.html> and
  <https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo3d.html>.
- Reference demo repositories:
  - `avbd-demo2d`: <https://github.com/savant117/avbd-demo2d>,
    inspected at `74699a11f8586d3ac34474c92b1ef8feb5f587de`.
  - `avbd-demo3d`: <https://github.com/savant117/avbd-demo3d>,
    inspected at `7701bd427d55ca5d03ea1fdf331912ded9169f4b`.

The paper positions AVBD as a hybrid primal-dual extension of VBD that adds a
dual update after each primal VBD sweep, supports hard constraints through an
augmented Lagrangian, improves high-stiffness-ratio convergence by ramping
finite forces, and evaluates rigid stacking/friction, articulated chains,
attachments, breakage, soft/rigid coupling, and large GPU scenes.

## Full-Scope Rule

AVBD is not complete when DART has only a small contact experiment or a
paper-inspired row update. For this paper, "implemented" means:

- all algorithms and features from the paper, project page, videos, and 2D/3D
  demo sources are implemented in DART-owned code;
- CPU and GPU implementations both exist for the solver, constraints, collision
  candidate generation, demos, and benchmark paths where the paper/reference
  exercises them;
- every paper/site/video/demo experiment is represented by a DART test,
  benchmark, and `py-demos` scene or a documented equivalent;
- DART benchmark JSON proves DART beats the reference demo repositories and the
  paper's reported CPU/GPU numbers for every claimed case; and
- API and pipeline refactors are allowed when they make the DART 7/8
  experimental architecture cleaner and more scalable.

## Solver-Identity Relabel (PLAN-091 WP-091.1)

No `avbd-demo2d`/`avbd-demo3d` benchmark or py-demo scene cited by this audit
emplaces the internal AVBD rigid-contact opt-in config
(`comps::RigidAvbdContactConfig`), because AVBD contact is not
facade-selectable, so every rigid contact in those scenes ran DART's default
sequential-impulse contact path. The native-runner timing ratios for contact
scenes are whole-pipeline `World::step` comparisons, not AVBD-contact-solver
comparisons: the pure-contact rows (2D Dynamic Friction, Static Friction,
Pyramid, Cards, Stack, and Stack Ratio; 3D Ground, Dynamic Friction, Static
Friction, Pyramid, Stack, and Stack Ratio) timed no AVBD rows at all; the
joint-plus-contact rows (2D Fracture, Soft Body, Joint Grid, and Net; 3D Soft
Body, Bridge, and Breakable) timed AVBD point-joint/motor/spring rows while
their ordinary contacts ran sequential impulse; and incidental link-link
contacts in the chain rows (2D Rod, Rope, Heavy Rope, and Hanging Rope; 3D
Rope and Heavy Rope) also ran sequential impulse. Faster-than-native rows in
the gap matrix and performance targets below carry this classification. This
relabel changes no committed packet bytes and neither closes nor reopens any
PLAN-104 completion gate; new AVBD evidence packets must machine-record
`resolved_solver_identity` at AVBD packet schema version 2, enforced by
`pixi run check-avbd-packets`.

## Verified Reference Details

- **Hard constraints:** Each scalar row carries finite stiffness `k` and dual
  value `lambda`. A hard row applies `clamp(k * C(x) + lambda, lower, upper)` as
  the force magnitude, then writes that value back to `lambda`.
- **Stiffness growth:** Hard rows increase `k` by `beta * abs(C)` only while the
  force is not clamped. Finite-stiffness forces use the same progressive ramp
  but saturate at the material stiffness and do not carry a Lagrange multiplier.
- **Bounds and friction:** Contacts and joint limits are inequality rows. The
  normal row has a non-negative lower bound. Friction rows are bounded by the
  lagged normal force and coefficient of friction; static/dynamic friction
  switching is based on whether the previous tangential dual lies inside the
  cone.
- **Error regularization:** Constraint values are regularized as
  `C(x) = C*(x) - alpha * C*(x_t)` so pre-existing hard-constraint error is
  corrected gradually instead of injecting a large impulse-like momentum spike.
- **Warm starting:** `lambda` and `k` persist across frames and decay by
  `alpha * gamma` and `gamma` respectively, with `k` never below `k_start`.
- **Hessian approximation:** Hard constraints use a quasi-Newton Hessian: the
  geometric stiffness term is replaced by a diagonal matrix built from column
  norms so each local block remains SPD.
- **Parallelization:** AVBD keeps VBD's vertex/body coloring, then adds one
  parallel dual/stiffness update pass after each primal iteration.
- **Rigid bodies:** Rigid rows use a 6-DOF block with linear and angular terms;
  the paper represents rotation by unit quaternions and maps rotational
  differences/updates through tangent-space angular vectors.
- **Reference demo sources:** `avbd-demo2d` implements boxes, joints, springs,
  motors, contact manifolds, fracture thresholds, and 19 scenes. `avbd-demo3d`
  implements 6-DOF rigid boxes, spring/joint/contact rows, bridge/breakable
  examples, and 14 scenes. Both demos state they are clarity references, not
  heavily optimized implementations.

## DART Current State

PLAN-104 already has a DART-owned VBD foundation: CPU block kernels, vertex
coloring, colored Gauss-Seidel sweeps, Stable Neo-Hookean/FEM tetrahedra,
Chebyshev/Rayleigh acceleration, opt-in World wiring, static ground/sphere/box
penalty contact, lagged surface self-contact, CUDA mass-spring/tet rollouts,
benchmarks, and first py-demos.

The first AVBD implementation slices add:

- a tested internal scalar-row utility for Eq. 11, Eq. 12, Eq. 16, Eq. 18, and
  Eq. 19: `detail/deformable_vbd/avbd_constraint.hpp` plus
  `test_avbd_constraint.cpp`;
- deterministic scalar-row descriptors, keys, role/axis separation, and
  warm-start inventory cache:
  `detail/deformable_vbd/avbd_row_inventory.hpp`;
- an active half-space contact-normal kernel slice:
  `addAvbdHalfSpaceContactNormal`, `updateAvbdHalfSpaceContactNormalRow`, and
  `blockDescentMassSpringAvbdGround`, with focused `VbdContact.Avbd*` tests;
- an active scalar hard point-attachment kernel slice:
  `AvbdPointAttachmentRow`, `addAvbdPointAttachment`,
  `updateAvbdPointAttachmentRow`, and
  `blockDescentMassSpringAvbdAttachments`, with focused `VbdAttachment.*`
  tests; and
- a narrow internal World VBD integration path for active static half-space
  contact-normal rows in supported serial mass-spring static-contact scenes,
  keyed by body/entity, vertex, and static obstacle feature IDs. Static box
  obstacle feature IDs distinguish faces, edges, and corners so normal/friction
  rows reset across box-manifold changes while warm-starting small same-feature
  penetrations, with `VbdContact.AvbdBoxContactFeatureCodeSeparatesBoxManifolds`
  coverage. Persisting static half-space friction rows project their decayed
  dual into the current tangent basis when smooth obstacle normals change, with
  `VbdContact.AvbdFrictionDualProjectionPreservesWorldImpulse` coverage plus
  `VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact` coverage; and
- a narrow internal World VBD integration path for pinned or scripted hard
  point-attachment rows in supported serial mass-spring scenes, keyed by
  body/entity, vertex, and axis, with per-step internal row counters and
  `VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode` coverage; and
- a narrow internal World VBD integration path for progressive finite-stiffness
  spring rows in supported serial mass-spring scenes, keyed by body/entity and
  spring index, with focused `VbdFiniteStiffness.*` tests and
  `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain` coverage; and
- a finite-stiffness tetrahedral material row slice:
  `AvbdTetMaterialFiniteStiffnessRow`, strain-norm row error, scaled Lamé
  material stamping, and `blockDescentTetMeshAvbdFiniteStiffness`, with focused
  `VbdFiniteStiffness.*` tests plus a narrow internal World VBD integration
  path for supported serial, frictionless pure-tet scenes, separate tet-row
  diagnostics, coexistence with the existing lagged VBD self-contact penalty,
  `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial` coverage,
  and
  `VbdWorldSolver.AvbdFiniteStiffnessRowsIgnoreUnusedFrictionCoefficient`
  coverage for contact-free finite-stiffness mass-spring scenes whose material
  friction coefficient has no active contact or self-contact source; and
- a combined serial mass-spring AVBD row solve for those contact-normal,
  attachment, and finite-stiffness spring rows, with
  `VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness`
  coverage; and
- a bounded half-space friction-tangent row primitive:
  `AvbdHalfSpaceFrictionRow`, `avbdFrictionTangentBounds`,
  `addAvbdHalfSpaceFrictionTangent`, and
  `updateAvbdHalfSpaceFrictionTangentRow`, with optional participation in the
  serial mass-spring AVBD row driver, supported World generation for static
  ground contact, focused `VbdContact.Avbd*` coverage, and
  `VbdWorldSolver.AvbdContactNormalRowsIncludeFrictionTangentRows` /
  `VbdWorldSolver.AvbdFrictionTangentRowsDecelerateSlidingBody` coverage;
  adjacent tangent-row pairs now use the lagged tangential dual for
  static/dynamic switching and project pair forces to the circular Coulomb cone,
  with `VbdContact.AvbdFrictionTangentPair*` coverage; persisting self-contact
  friction rows project their generalized tangential dual into the current 12D
  tangent stencil with
  `VbdCombinedDescent.AvbdSelfContactFrictionDualProjectionPreservesGeneralizedImpulse`
  coverage; private rigid contact snapshots now map known and unknown-index
  contacts through body and shape transforms before feature coding, including
  endpoint-A/B explicit compound shape-index coverage that uses narrow-phase
  shape-local contact points and actual `World::collide()`
  sphere/cylinder/capsule/plane/mesh primitive-feature coverage including live
  mesh-vertex endpoint-feature evidence; live
  sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
  mesh-face/mesh-edge/mesh-vertex small-pose persistence plus endpoint-order
  stability,
  positive/negative cylinder-cap/plane and capsule-cap/plane same-feature
  row-order coverage, live cylinder-side/capsule-side and positive/negative cylinder-rim
  row-order coverage, cylinder/capsule cap/side/rim small-pose persistence and
  endpoint-order stability,
  live sphere/plane friction tangent warm-start mapping,
  live sphere/plane normal-rotation friction tangent projection,
  live endpoint-swapped box-box friction tangent projection, live box-box
  same-feature row-order, endpoint-order, small-pose row-persistence, and
  manifold friction warm-start coverage, plus a live
  stacked static/dynamic and dynamic/dynamic box-manifold small-pose check and
  stacked friction warm-start plus endpoint-swapped tangent projection checks,
  with multi-top stacked contact-stage/default-step friction slip-regressions,
  wider spanning-top box-pile contact-stage/default-step friction
  slip-regressions, and two-lower/two-upper box-pile friction row-persistence,
  contact-order replay, endpoint-order tangent projection, and
  contact-stage/default-step friction slip-regressions now protect the first
  realistic manifold slices; private
  rigid contact
  friction rows can now project their
  warm-started paired tangent dual into the current basis when contact normals
  rotate, with both private row-builder and live sphere/plane coverage, while
  broad dynamic/rigid contact manifold persistence is still
  missing; and
- a self-contact normal row slice:
  `AvbdSelfContactNormalRow`, `avbdSelfContactNormalConstraintValue`,
  `addAvbdSelfContactNormal`, and `updateAvbdSelfContactNormalRow`, using the
  IPC point-triangle / edge-edge barrier direction for AVBD hard-row stamping,
  with focused `VbdCombinedDescent.AvbdSelfContact*` coverage plus narrow
  supported serial mass-spring World generation, a separate
  `vbdAvbdSelfContactNormalRows` diagnostic counter, and
  `VbdWorldSolver.AvbdSelfContactNormalRowsPushSupportedSurfaceApart`
  coverage; and
- a standalone self-contact friction row slice:
  `AvbdSelfContactFrictionRow`,
  `addAvbdSelfContactFrictionTangent`,
  `updateAvbdSelfContactFrictionTangentRow`, and pairwise cone helpers, using
  the IPC point-triangle / edge-edge tangent stencils against lagged primitive
  positions with supported World generation in serial mass-spring self-contact
  scenes and focused `VbdCombinedDescent.AvbdSelfContactFriction*` plus
  `VbdWorldSolver.AvbdSelfContactNormalRowsIncludeFrictionTangentRows`
  and `VbdWorldSolver.AvbdSelfContactFrictionRowsReduceTangentialMotion`
  coverage, plus `VbdWorldSolver.AvbdContactAndSelfContactFrictionRowsCombine`
  coverage for combined static-contact and self-contact friction row
  coexistence in one supported solve; and
- a pure-tetrahedral AVBD self-contact envelope:
  `blockDescentTetMeshAvbdFiniteStiffness` can now take AVBD self-contact
  normal rows and matching self-contact friction tangent rows alongside
  finite-stiffness material rows, so supported serial pure-tet World scenes can
  report tet finite-stiffness, self-contact normal, and self-contact friction
  row families in one solve, with
  `VbdWorldSolver.AvbdTetRowsCombineSelfContactFrictionRows` coverage; and
- a private 6-DOF rigid-body block foundation:
  `AvbdRigidBodyBlock`, world-frame quaternion tangent-step helpers,
  `addAvbdRigidBodyInertiaTerm`, `solveAvbdRigidBodyBlock`,
  `applyAvbdRigidBodyStep`, `AvbdRigidPointAttachmentRow`, and
  `AvbdRigidPointPairRow`, with focused `AvbdRigidBlock.*` coverage. Rigid
  point-pair rows now include a scalar offset plus private rigid contact-normal
  and bounded contact-friction tangent constructors, and paired tangent rows can
  switch between static sticking and dynamic sliding while projecting the force
  to a circular Coulomb cone. A private serial rigid row driver can sweep point
  attachments, contact-normal point pairs, and paired friction tangent rows.
  A private rigid contact-manifold row builder can map active contact points
  with stable endpoint feature IDs into warm-started normal and paired tangent
  rows for that driver. Persisting private rigid friction rows retain their
  previous tangent directions and project the warm-started paired dual into the
  current tangent basis when the contact normal rotates. A private
  World-contact snapshot helper can translate
  rigid-body `World::collide()` contacts into those manifold-point inputs for
  focused coverage through the private serial rigid row solve and dynamic
  rigid-body ECS writeback, including a combined private build/solve/writeback
  wrapper. The public fixed-joint path now has a first AVBD-specific
  `py-demos` scene, `avbd_rigid_fixed_joint_contact`, showing fixed rigid rows
  preserving a captured offset while ordinary rigid contact acts on the
  payload. This is still a narrow foundation only; full solver-stage rigid
  contact activation, articulated joints, motors, fracture, soft/rigid
  coupling, and paper/demo corpus reproduction are still missing; and
- explicit World fallback coverage so unsupported mixed spring-plus-tet,
  mass-spring self-contact without the self-contact AVBD flag, Chebyshev,
  Rayleigh-damped, parallel, and unsupported-row requests keep using the
  existing VBD path without reporting partial AVBD row counters.

Those are foundation pieces only. They are not a full World AVBD solver, not a
full hard-contact/friction solver, and not CPU/GPU parity.

## Component Gap Matrix

| AVBD component                                                                                     | DART status                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Closing phase |
| -------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| Scalar hard-row dual state, clamping, warm start, alpha regularization, finite-stiffness ramp      | First internal utility and unit tests started                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | A0            |
| Constraint row inventory and storage for contacts, joints, attachments, friction, motors, fracture | Deterministic scalar row inventory started; active static contact-normal, hard-attachment, finite-spring, friction-tangent, self-contact-normal, and self-contact-friction World rows wired in narrow envelopes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | A1            |
| CPU AVBD primal/dual loop over DART VBD vertices and rigid bodies                                  | Narrow serial mass-spring contact-normal, hard-attachment, finite-spring, optional friction-tangent, self-contact-normal, and self-contact-friction rows now combine; pure-tet finite-material rows now combine with AVBD self-contact normal/friction rows when requested; private serial rigid row driver started for point attachments, contact-normal point pairs, paired friction tangents, and rows built from active rigid contact-manifold points; private World-contact snapshot extraction now feeds rigid collision contacts into that row-builder, serial row-solve, and dynamic rigid-body ECS writeback input in focused tests through a combined private wrapper                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | A2-A4         |
| 6-DOF rigid block assembly with quaternion/tangent angular updates                                 | Private 6-DOF block accumulator, inertia term, quaternion tangent update, block solve, scalar point-attachment row, two-body point-pair row stamping, serial rigid row driver, contact-manifold row builder, and World-contact snapshot/solve/writeback helpers plus combined private wrapper started; point-pair contact-normal and bounded contact-friction tangent row construction plus paired friction-cone projection exist as private kernel targets; initial `RigidAvbdContactConfig` contact-stage activation exists for supported free rigid-body contacts and now has focused dynamic/dynamic and static/dynamic contact-stage velocity-projection coverage plus mixed-config all-or-nothing fallback and warm-started friction slip-reduction coverage, while broad feature persistence, joints, motors, and articulation remain open                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | A3            |
| Inequality/contact/friction row bounds with static/dynamic friction switching                      | Contact-normal lower-bound slice and bounded friction-tangent World generation started for supported static mass-spring contact; adjacent deformable tangent pairs and private rigid point-pair tangent rows have static/dynamic switching and pairwise cone projection; full contact-manifold cone persistence missing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | A4            |
| Joint, attachment, motor, fracture, and breakable hard constraints                                 | Scalar hard point-attachment kernel and narrow World attachment wiring started; private fixed-joint row builders now cover all-axis fixed translation/orientation rows plus masked linear/angular axes; named private revolute/prismatic point-joint configs now build arbitrary joint-axis bases, preserve axes/masks through World point-joint input and solve coverage, and initialize/extract from private rigid-body ECS one-DOF joint entities at simulation entry; non-topology multibody-link fixed/revolute/prismatic point-joint entities can now also generate hard private AVBD configs from the simulation-entry current pose while tree-topology joints are skipped; public DART 7 `World` facades now expose free rigid-body revolute/prismatic joints through C++/dartpy with tests and a `sx_rigid_limited_joints` py-demo; public free-rigid-body revolute/prismatic velocity actuators now extract to private bounded AVBD angular/linear motor rows, with a categorized py-demo plus one end-to-end dashboard benchmark row and packet for the revolute path and focused C++/dartpy stepping coverage for the prismatic path; public articulated revolute/prismatic velocity motors and the active break-force armed articulated motor path now also have narrow dashboard benchmark rows; a narrow public free-rigid-body break-force/broken-state lifecycle now marks solved AVBD point joints broken, skips later extraction, and has an `avbd_rigid_breakable_joint` py-demo with reset/re-engagement coverage plus an `avbd_rigid_spherical_breakable_joint` py-demo for spherical anchor-only reset/re-engagement; explicitly hard fixed and masked revolute/prismatic multibody-link point-joint configs now bridge into the variational articulated solve path, breakable hard configs mark the source joint broken from projection load with fixed mark/reset, masked-revolute, and velocity-motor coverage, public one-DOF articulated `Velocity` actuators project as variational coordinate motor targets, private hard revolute/prismatic AVBD point-joint velocity actuators now project one bounded free-axis motor row for articulated endpoints with revolute/prismatic command-update, private fixed-row reset, revolute/prismatic break/reset re-engagement, and tiny positive effort-limit coverage, finite-stiffness private AVBD fixed point-joint configs on articulated endpoints now contribute compliant variational forces through persistent stiffness-ramped rows, and public same-multibody/world-link articulated fixed/revolute/prismatic/spherical facades now feed the current-pose extractor through C++/dartpy with focused spherical linear-only pinned-anchor behavior including explicit link-link/world-link anchors, bounded revolute/prismatic velocity-actuator, same-multibody/world-anchored command-update, same-multibody/world-anchored public one-DOF motor break/skip with focused dartpy stepping coverage for same-multibody/world-link revolute and prismatic explicit-anchor break/skip and reset/re-engagement cases plus non-cardinal same-multibody pair and world-link revolute/prismatic reset regressions plus C++/dartpy non-cardinal off-origin finite-effort cap checks spanning same-multibody/world-link revolute and prismatic facades, same-multibody/world-anchored public one-DOF motor break/reset re-engagement, same-multibody revolute/prismatic and world-anchored prismatic/revolute break/reset py-demo coverage, movable-movable same-multibody motor projection, explicit local/world anchor projection, same-multibody fixed and world-fixed break/reset, same-multibody fixed break/reset py-demo coverage, spherical linear-row break/reset py-demo coverage for world-link and same-multibody endpoints, same-multibody link-link and world-link fixed/spherical save/load rebuilding of the private all-axis and linear rows, same-multibody/world-link revolute/prismatic motor save/load rebuilding of the private hard rows and free-axis motor row, same-multibody/world-link fixed/spherical/revolute/prismatic broken-state save/load/reset persistence, dartpy fixed point-joint break/skip/reset stepping for same-multibody and world-link explicit all-axis anchor rows, and world-anchor coverage; broader public articulated facade coverage beyond those link-link/world-link/anchor/spherical entrypoints and save/load rebuilding, broader persistent private articulated motor coverage beyond those command-update, movable-pair, and one-DOF break/skip/break-reset checks, and broad breakable-constraint coverage remain missing | A5            |
| Unified rigid/soft interactions and cloth/articulated-body coupling                                | Missing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | A6            |
| Collision candidate generation, contact persistence, and row warm-start mapping                    | Static half-space row keys include static box face/edge/corner feature IDs and smooth-frame tangent projection, lagged self-contact normal/friction candidate row keys plus self-contact tangent projection started for mass-spring and pure-tet envelopes, and private dynamic/rigid contact feature IDs plus canonical two-endpoint row keys, normal/friction row descriptor helpers, rigid point-pair contact/friction constructors, a private rigid contact-manifold row builder, private World-contact snapshot/solve/writeback extraction plus combined wrapper, initial contact-stage activation with focused dynamic/dynamic and static/dynamic velocity-projection coverage plus mixed-config all-or-nothing fallback and warm-started friction slip-reduction coverage including dynamic-owned, static-owned, kinematic-owned, enabled-peer/disabled-peer, simultaneous multi-contact, static/dynamic box-manifold, dynamic/dynamic box-manifold, stacked static/dynamic plus dynamic/dynamic box-manifold, multi-top stacked box-manifold, default-step dynamic/dynamic, stacked, and multi-top stacked box-manifold schedule paths, and dynamic/dynamic sphere focused regressions, box/sphere/cylinder/capsule endpoint feature persistence, known/unknown shape-frame feature mapping from world contact points including endpoint-A/B explicit compound shape-index coverage that uses narrow-phase shape-local contact points, actual `World::collide()` sphere/cylinder/capsule/plane/mesh primitive-feature coverage including shape-scoped sphere body features, plane face feature persistence, mesh face/edge/vertex feature persistence, deterministic same-feature rigid contact row ordering including actual `World::collide()` sphere/plane contact-point replay, sphere/mesh-face contact-point replay, sphere/mesh-edge contact-point replay, mesh-vertex replay, mesh-face/mesh-edge/mesh-vertex small-pose persistence plus endpoint-order stability, cylinder-cap/plane contact-point replay, capsule-cap/plane contact-point replay, cylinder-side/capsule-side contact-point replay, cylinder-rim contact-point replay, and cylinder/capsule cap/side/rim small-pose persistence plus endpoint-order stability, live sphere/plane friction tangent warm-start mapping, live sphere/plane normal-rotation friction tangent projection evidence, live sphere/plane and box-box manifold endpoint-order row-identity evidence, live endpoint-swapped box-box friction tangent projection evidence, live box-box small-pose row-persistence evidence, live box-box manifold friction warm-start evidence, live stacked-box friction warm-start and endpoint-swapped friction tangent projection evidence, live spanning-top and multi-top box-pile friction row-persistence, contact-order replay, and endpoint-swapped tangent projection evidence plus contact-stage/default-step friction slip-reduction evidence, and live box-box manifold box-face feature evidence, private rigid contact tangent-dual projection across tangent-basis changes, and the first private multibody-link endpoint projection bridge started; broader contact-stage dynamic-contact coverage, broad articulated endpoint mapping, broad feature persistence, and broad row generation remain open                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | A7            |
| Hessian approximation for hard constraints and stiffness-rescaling friction                        | Finite-stiffness spring and pure-tet material ramps started; hard-constraint quasi-Newton Hessian and friction stiffness rescaling missing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | A8            |
| CPU parallel color sweeps plus deterministic dual update pass                                      | Serial contact-normal dual pass started; parallel dual/update scheduling missing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | A9            |
| CUDA/GPU AVBD backend for all row families and scene corpus                                        | VBD CUDA mass-spring/tet rollout only                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | G1-G5         |
| DART-owned reproductions of 2D/3D demos and paper/video scenes                                     | First narrow AVBD rigid-constraint py-demo started (`avbd_rigid_fixed_joint_contact` for fixed-joint rows plus ordinary contact), `avbd_rigid_revolute_motor` covers the public free-rigid-body velocity-motor row path, `avbd_articulated_revolute_motor` and `avbd_articulated_prismatic_motor` cover narrow public articulated velocity-motor command-update paths, `avbd_articulated_motor_breakable_joint` covers a narrow same-multibody public articulated revolute motor break/reset lifecycle, `avbd_articulated_prismatic_pair_motor_breakable_joint` covers a narrow same-multibody public articulated prismatic motor break/reset lifecycle, `avbd_articulated_prismatic_motor_breakable_joint` covers a narrow world-anchored public articulated prismatic motor break/reset lifecycle, `avbd_articulated_world_revolute_motor_breakable_joint` covers a narrow world-anchored public articulated revolute motor break/reset lifecycle, `avbd_rigid_breakable_joint` covers the narrow public free-rigid-body break-force lifecycle, `avbd_rigid_spherical_breakable_joint` covers the narrow public free-rigid-body spherical break/reset lifecycle, `avbd_articulated_breakable_joint` covers the narrow public world-link articulated fixed point-joint break/reset lifecycle, `avbd_articulated_fixed_pair_breakable_joint` covers the narrow public same-multibody articulated fixed point-joint break/reset lifecycle, `avbd_articulated_spherical_breakable_joint` covers the narrow public world-link articulated spherical break/reset lifecycle, `avbd_articulated_spherical_pair_breakable_joint` covers the narrow public same-multibody articulated spherical break/reset lifecycle, `avbd_articulated_high_ratio_chain`, `BM_AvbdArticulatedHighRatioChainStep`, and [`avbd-articulated-high-ratio-chain-packet.json`](avbd-articulated-high-ratio-chain-packet.json) cover a narrow five-link 200:1 high mass-ratio articulated-chain smoke, `avbd_paper_scale_high_ratio_chain`, `BM_AvbdPaperScaleHighRatioChainStep`, and [`avbd-paper-scale-high-ratio-chain-packet.json`](avbd-paper-scale-high-ratio-chain-packet.json) add 50-link/50,000:1 visual and CPU benchmark evidence without a same-hardware comparison, `avbd_empty_baseline` provides a shared smoke baseline plus source-row metadata/reference invariant for the 2D/3D source-demo empty rows with a tracked visual/benchmark packet, `avbd_demo2d_ground`, `avbd_demo2d_motor`, `avbd_demo2d_hanging_rope`, `avbd_demo2d_fracture`, `avbd_demo2d_dynamic_friction`, `avbd_demo2d_static_friction`, `avbd_demo2d_pyramid`, `avbd_demo2d_cards`, `avbd_demo2d_stack`, `avbd_demo2d_stack_ratio`, `avbd_demo2d_rod`, `avbd_demo2d_soft_body`, `avbd_demo2d_joint_grid`, `avbd_demo2d_rope`, `avbd_demo2d_net`, `avbd_demo3d_ground`, `avbd_demo3d_dynamic_friction`, `avbd_demo3d_static_friction`, `avbd_demo3d_pyramid`, `avbd_demo3d_rope`, `avbd_demo3d_heavy_rope`, `avbd_demo3d_spring`, `avbd_demo3d_spring_ratio`, `avbd_demo3d_stack`, `avbd_demo3d_stack_ratio`, `avbd_demo3d_soft_body`, `avbd_demo3d_bridge`, and `avbd_demo3d_breakable` now provide the first non-empty source-row ports with focused invariants, dashboard rows, and tracked source timing packets, and `sx_rigid_limited_joints` covers the public one-DOF joint facade path; paper figures, parameter sweeps, website demos, video/headline scenes, broad GPU evidence, benchmark-linked reproductions, and matched source-demo CPU wins remain missing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | D1-D6         |
| Benchmark packets beating reference demos and paper numbers on CPU/GPU                             | Narrow CPU dashboard rows now track the public free-rigid-body revolute velocity-motor AVBD path, public articulated revolute/prismatic velocity-motor paths, public free-rigid/articulated breakable fixed point-joint paths, the narrow five-link 200:1 high-ratio articulated-chain smoke, the 50-link/50,000:1 paper-scale high-ratio dashboard row, the `avbd-demo2d` Ground, Motor, Hanging Rope, Fracture, Dynamic Friction, Static Friction, Pyramid, Cards, Stack, Stack Ratio, Rod, Soft Body, Joint Grid, Rope, Heavy Rope, and Net source rows, the `avbd-demo3d` Ground, Dynamic Friction, Static Friction, Pyramid, Rope, Heavy Rope, Spring, Spring Ratio, Stack, Stack Ratio, Soft Body, Bridge, and Breakable source rows, and empty source-demo baseline alongside the fixed-joint row; the narrow high-ratio chain smoke, empty source-demo baseline, `avbd-demo2d` Ground/Motor/Hanging Rope/Fracture/Dynamic Friction/Static Friction/Pyramid/Cards/Stack/Stack Ratio/Rod/Soft Body/Joint Grid/Rope/Heavy Rope/Net rows, and `avbd-demo3d` Ground, Dynamic Friction, Static Friction, Pyramid, Rope, Heavy Rope, Spring, Spring Ratio, Stack, Stack Ratio, Soft Body, Bridge, and Breakable rows have tracked packets, with the soft-body and Joint Grid packets configuring the source diagonal ignore-collision pairs through DART's public per-pair filter; the paper-scale high-ratio row now has a visual/benchmark packet but still lacks a same-hardware paper-number comparison; 2D Ground, 2D Fracture, 2D Dynamic Friction, 2D Static Friction, 2D Pyramid, 2D Stack, 2D Stack Ratio, 3D Ground, 3D Dynamic Friction, 3D Static Friction, Pyramid, Stack, Stack Ratio, Soft Body, Bridge, and Breakable are faster than the native source runner on this host but Motor, Hanging Rope, 2D Cards, 2D Rod, 2D Soft Body, 2D Joint Grid, 2D Rope, 2D Heavy Rope, 2D Net, 2D/3D Spring, 2D/3D Spring Ratio, 3D Rope, and 3D Heavy Rope are still slower, so broad CPU/GPU parity and proof against published paper numbers remain missing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | P1-P4         |

Contact-stage note:
`World.RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig` verifies
that dynamic/dynamic contact-stage AVBD activation works when only one dynamic
endpoint carries the private opt-in config.
`World.RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep`
verifies that both-configured dynamic/dynamic projection feeds through the
built-in `World::step()` schedule and following rigid position stage.
`World.RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep`
verifies the same default schedule path when only one dynamic endpoint carries
the private opt-in config.
`World.RigidBodyContactStageAvbdFeedsRigidBodyPositionStage` verifies that the
contact-stage AVBD velocity projection feeds the following rigid position stage
instead of relying on direct pose writeback.
`World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep` verifies that the
built-in `World::step()` schedule reaches the same AVBD contact-stage velocity
projection and following rigid position stage.
`World.RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig` verifies
that static/dynamic contact-stage AVBD activation works when the private opt-in
config is owned by the static contact endpoint rather than the dynamic endpoint.
`World.RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep`
verifies that the built-in `World::step()` schedule reaches the same
static-owned activation path.
`World.RigidBodyContactStageAvbdIgnoresStoredStaticVelocity` verifies that a
stored static-body velocity does not perturb the static-owned AVBD contact-stage
projection compared with an otherwise identical zero-static-velocity baseline.
`World.RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle` verifies
that kinematic endpoints stay fixed/prescribed in the AVBD contact snapshot and
that their prescribed velocity does not perturb the dynamic body's contact-stage
projection.
`World.RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep` verifies
that the same kinematic prescribed-endpoint behavior feeds through the built-in
`World::step()` schedule and following rigid position stage.
`World.RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig` verifies
that an explicitly disabled peer config does not veto AVBD activation when the
other contact endpoint carries an enabled private config.
`World.RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep`
verifies that the built-in `World::step()` schedule preserves the same
enabled-peer/disabled-peer activation.
`World.RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts` verifies
that the internal contact-stage AVBD path projects a live configured contact set
with two static/dynamic contacts, not only single-contact manifolds.
`World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep`
verifies that configured multi-contact projection feeds through the built-in
`World::step()` schedule and following rigid position stage.
`World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet` verifies
that a mixed configured/unconfigured live contact set falls back as a whole
instead of partially activating AVBD for only the configured contacts.
`World.RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep`
verifies that the built-in `World::step()` schedule preserves the same
all-or-nothing fallback.
`World.RigidBodyContactStageAvbdDisabledConfigFallsBack` verifies that an
explicitly disabled private contact config opts out to the ordinary rigid
contact response instead of activating the AVBD projection.
`World.RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep`
verifies that the built-in `World::step()` schedule preserves the same disabled
opt-out fallback.
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip` extends
the warm-started friction evidence from static/dynamic sphere-ground contact to
dynamic/dynamic sphere contact by checking reduced relative tangential
contact-point slip against a frictionless baseline.
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide`
verifies the same warm-started friction path when the static contact endpoint
owns the private opt-in config.
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`
verifies the adjacent kinematic-owned warm-started friction path and checks
that a prescribed kinematic tangential velocity is preserved but ignored as
contact slip input.
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`
verifies that an explicitly disabled peer config does not veto warm-started
friction rows when the other endpoint carries an enabled private config.
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`
verifies that simultaneous configured static/dynamic contacts both feed
warm-started friction rows in one contact-stage solve.
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`
verifies that live box-box manifolds with multiple narrow-phase contact points
feed warm-started contact-stage friction rows.
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`
extends that manifold evidence to a live dynamic/dynamic box-box contact-stage
solve and verifies reduced relative contact-point slip against a frictionless
baseline.
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip`
extends the contact-stage stack evidence to two independent dynamic/dynamic
upper contacts sharing one supported lower body.
`World.RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep`
and
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`
verify that the built-in `World::step()` schedule reaches the dynamic-owned,
static-owned, kinematic-owned, enabled-peer/disabled-peer, and simultaneous
multi-contact static/dynamic, box-manifold static/dynamic, and dynamic/dynamic
box-manifold plus multi-top stacked box-manifold warm-started friction paths.
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep`
verifies that the built-in `World::step()` schedule reaches the dynamic/dynamic
warm-started friction path. This is still narrow contact-stage evidence, not
broad contact-stage, CPU/GPU, or paper-number parity.

Note: [`avbd-rigid-revolute-motor-packet.json`](avbd-rigid-revolute-motor-packet.json)
now records the public free-rigid revolute motor row's headless visual capture
and `BM_AvbdRigidRevoluteMotorStep/1` benchmark.
[`avbd-rigid-prismatic-motor-packet.json`](avbd-rigid-prismatic-motor-packet.json)
now records the public free-rigid prismatic motor row's headless visual capture
and `BM_AvbdRigidPrismaticMotorStep/1` benchmark.
[`avbd-articulated-revolute-motor-packet.json`](avbd-articulated-revolute-motor-packet.json)
now records the public articulated revolute motor row's headless visual capture
and `BM_AvbdArticulatedRevoluteMotorStep/1` benchmark.
[`avbd-articulated-prismatic-motor-packet.json`](avbd-articulated-prismatic-motor-packet.json)
now records the public articulated prismatic motor row's headless visual capture
and `BM_AvbdArticulatedPrismaticMotorStep/1` benchmark.
[`avbd-articulated-breakable-motor-packet.json`](avbd-articulated-breakable-motor-packet.json)
now records the public same-multibody articulated revolute motor break/reset
scene's headless visual capture and `BM_AvbdArticulatedBreakableMotorStep/1`
benchmark. Focused integration coverage now also reverses its target velocity
after reset, verifies the re-engaged motor follows the updated command, and
checks weak re-arm breaks again.
[`avbd-articulated-prismatic-pair-breakable-motor-packet.json`](avbd-articulated-prismatic-pair-breakable-motor-packet.json)
now records the public same-multibody articulated prismatic motor break/reset
scene's headless visual capture and
`BM_AvbdArticulatedPrismaticBreakableMotorStep/1` benchmark. Focused
integration coverage now also reverses its target velocity after reset and
checks weak re-arm breaks again.
[`avbd-articulated-world-prismatic-breakable-motor-packet.json`](avbd-articulated-world-prismatic-breakable-motor-packet.json)
now records the public world-anchored articulated prismatic motor break/reset
scene's headless visual capture and
`BM_AvbdArticulatedWorldPrismaticBreakableMotorStep/1` benchmark. Focused
integration coverage now also reverses its target velocity after reset and
checks weak re-arm breaks again.
[`avbd-articulated-world-revolute-breakable-motor-packet.json`](avbd-articulated-world-revolute-breakable-motor-packet.json)
now records the public world-anchored articulated revolute motor break/reset
scene's headless visual capture and
`BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/1` benchmark. Focused
integration coverage now also reverses its target velocity after reset and
checks weak re-arm breaks again. Broad motor lifecycle coverage, GPU parity, and
paper-number gates remain open.
[`avbd-rigid-breakable-joint-packet.json`](avbd-rigid-breakable-joint-packet.json)
and
[`avbd-articulated-breakable-joint-packet.json`](avbd-articulated-breakable-joint-packet.json)
now record the public free-rigid and world-link articulated fixed point-joint
break/reset scenes' headless visual captures plus
`BM_AvbdRigidBreakableJointStep/1` and
`BM_AvbdArticulatedBreakableJointStep/1` benchmark evidence. Focused demo
regressions also verify weak re-arm breaks again after high-force reset for
the free-rigid and world-link fixed paths.
[`avbd-articulated-fixed-pair-breakable-joint-packet.json`](avbd-articulated-fixed-pair-breakable-joint-packet.json)
records the same-multibody articulated fixed point-joint break/reset scene's
headless visual capture against the same `/1` benchmark row, and focused demo
coverage verifies weak re-arm breaks again after reset.
[`avbd-rigid-spherical-breakable-joint-packet.json`](avbd-rigid-spherical-breakable-joint-packet.json),
[`avbd-articulated-spherical-breakable-joint-packet.json`](avbd-articulated-spherical-breakable-joint-packet.json),
and
[`avbd-articulated-spherical-pair-breakable-joint-packet.json`](avbd-articulated-spherical-pair-breakable-joint-packet.json)
now record the public free-rigid, world-link articulated, and same-multibody
articulated spherical break/reset scenes' headless visual captures plus
`BM_AvbdRigidSphericalBreakableJointStep/1`,
`BM_AvbdArticulatedWorldSphericalBreakableJointStep/1`, and
`BM_AvbdArticulatedSphericalPairBreakableJointStep/1` benchmark evidence.
Focused demo regressions also verify weak re-arm breaks again after high-force
reset for all three spherical paths. Broad
fracture-constraint, GPU parity, and paper-number gates remain open.

Note: `avbd_demo2d_net`, `BM_AvbdDemo2dNetStep`, and
`avbd-demo2d-net-packet.json` now cover the source Net row shape/count,
visual-capture, DART benchmark, and native timing evidence. The packet records
DART about 1.66x slower than the native Net runner on this host after caching
snapshot body indices, so the CPU-win gate remains open.

Note: `avbd_demo2d_soft_body`, `BM_AvbdDemo2dSoftBodyStep`, and
`avbd-demo2d-soft-body-packet.json` now cover the 2D source Soft Body row's
shape/count, finite linear/angular joint stiffness, diagonal ignored-collision
pairs, visual-capture, DART benchmark, and native timing evidence. The packet
records DART about 6.30x slower than the native Soft Body runner on this host,
so the CPU-win gate, GPU parity, and paper-number gates remain open.

Note: the rigid AVBD contact stage now reuses a scratch contact snapshot through
`buildAvbdRigidWorldContactSnapshotInto`, avoiding per-step contact-snapshot
container recreation in contact-enabled rows, and now skips pair-constraint
input extraction when no point-joint or distance-spring configs exist.
The kinematics stage now also reuses its cached graph until the world's
frame-topology revision changes, avoiding a prepare-time graph rebuild on
unchanged source rows while preserving stale-topology refresh coverage.
The split rigid-body velocity stage also now assembles force batches only for
advanceable bodies.
The rigid row driver now also keeps unchanged source-row index layouts warm
across frames and routes single-family point-pair/angular solves without
rebuilding combined row vectors, while one-new-row point-joint/distance-spring
appends skip endpoint row-counter hash-map setup. The contact stage also
extracts/appends point-joint and distance-spring families independently when
only one family exists, and the snapshot solve clears absent row-family
inventories directly instead of calling empty contact/joint/motor/spring
builders. Small point-joint/motor/distance-spring row builders also now keep
descriptor/active-row temporaries on the stack for up to 16 candidate rows.
Large rigid World pair-constraint snapshots now activate their reserved
entity-to-body index map immediately, avoiding repeated early-body linear lookup
in large rope/net/Joint Grid/soft-body source rows.
Rigid AVBD block assembly now also reuses already-computed point-pair and
distance-spring world anchors/axes while stamping each body's row
force/Hessian, avoiding repeated local-to-world point transforms inside the
projection loop. The same loop reuses the shared angular orientation error
across consecutive all-axis angular rows, avoiding repeated quaternion error
evaluation for fixed-joint angular triplets during block assembly and row-state
updates. Finite-stiffness point-joint row builders now skip the unused
previous-constraint capture, and the point-pair block/update loops reuse
consecutive shared world-anchor transforms for fixed-joint linear triplets.
Friction tangent-pair row-state updates now also reuse the
precomputed regularized tangent constraint vector for force projection and
stiffness growth, and shared-anchor tangent pairs reuse the first transformed
anchor pair in the generic force helper, direct block assembly, and row-state
update path. Capped rigid finite-stiffness point-pair, angular-pair, and
distance-spring rows now also return from their row-state update without
recomputing current constraint geometry once the effective stiffness is already
at the material cap, matching the public fixed-stiffness cases. Local selected-row profiling reports the Joint Grid contact slice at
about 14.0 ms and Soft Body at about 3.0 ms after the row-assembly reuse, and
current selected-row benchmark smoke after angular-row reuse reports Joint Grid
at about 10.1 ms, Soft Body at about 2.14 ms, Rope at about 0.120 ms, and Net
at about 0.389 ms per step on this host. Current small-row benchmark smoke also
reports Motor median around 8.43 us, Spring median around 3.98 us, and Spring
Ratio median around 36.16 us, and refreshed Spring/Spring Ratio packets now
record about 3.82 us 2D Spring, 31.8 us 2D Spring Ratio, 3.58 us 3D Spring,
and 29.1 us 3D Spring Ratio while keeping those CPU-win gates open. The
friction row-state update smoke reports
Dynamic Friction around 14.2 us, Static Friction around 5.51 us, and Cards
around 0.472 ms under load average around 7.5. The shared-anchor reuse smoke
reports Dynamic Friction around 22.9 us, Static Friction around 5.99 us, Cards
around 0.507 ms, and Net around 0.433 ms under load average around 4.3. A
later friction-pair reuse diagnostic reported about 0.550 ms Cards,
10.7 ms Joint Grid, 2.43 ms Soft Body, 0.135 ms Rope, and 0.442 ms Net under
load, and the later point-pair anchor reuse smoke reports Soft Body median
around 2.03 ms, with Rod, Joint Grid, Soft Body, Rope, and Net around
0.149 ms / 10.8 ms / 1.70 ms / 0.113 ms / 0.360 ms in a selected-row run.
The later capped finite angular update cleanup keeps row-state updates from
recomputing orientation error once finite angular stiffness is already clamped;
its selected source-row benchmark smoke ran under high host load and is only
local overhead evidence.
The later exact-cache-hit cleanup replaces zero-tolerance approximate
comparisons for repeated point-pair anchors and angular targets in the rigid
block solver; its selected source-row benchmark smoke also ran under high host
load and is only local overhead evidence.
These remain local overhead diagnostics rather than refreshed CPU-win packet
evidence. A broader capped finite-row
diagnostic over Rod, Joint Grid, Soft Body, and Spring rows ran under load
average around 11.7 and is also non-packet overhead smoke.
Same-command benchmark smoke over selected 2D contact-heavy source rows was
mixed and host-load sensitive, so this is overhead cleanup only; no packet
CPU-win gate is closed by this note.

Note: `avbd_demo3d_soft_body`, `BM_AvbdDemo3dSoftBodyStep`, and
`avbd-demo3d-soft-body-packet.json` now cover the 3D source Soft Body row's
shape/count, finite linear/angular joint stiffness, diagonal ignored-collision
pairs, visual-capture, DART benchmark, and native timing evidence. The packet
records DART about 1.21x faster than the native Soft Body runner on this host,
so the narrow CPU gate is closed while GPU parity and paper-number gates remain
open.

## Implementation Workstreams

1. **A0 - scalar row foundation:** Land the internal AVBD scalar-row utility,
   focused tests for hard rows, finite-stiffness rows, bounds, warm starting,
   and alpha regularization.
2. **A1 - row model and inventory:** Add internal row descriptors for hard
   equality, bounded inequality, finite stiffness, contact normal, friction
   tangent, joint, motor, and fracture rows. Keep public API method names
   DART-owned and backend-neutral.
3. **A2 - CPU deformable AVBD:** Integrate scalar rows into the existing VBD
   deformable block descent for hard attachments, finite-stiffness ramped
   springs/tets, ground/static obstacle contact, and self-contact.
4. **A3 - CPU rigid AVBD:** Add 6-DOF rigid block assembly, tangent angular
   update, box inertia, rigid contact manifolds, and row Jacobians.
5. **A4 - friction and bounds:** Implement contact normal bounds, tangential
   friction cone bounds, static/dynamic friction switching, and stiffness
   rescaling/approximate Hessian options.
6. **A5 - joints, motors, and fracture:** Implement ball/revolute/limited DOF
   joints, motor torque rows, attachment rows, maximum-force fracture, and
   breakage persistence.
7. **A6 - unified rigid/soft coupling:** Couple rigid rows and deformable VBD
   rows in one AVBD solve so soft cloth/elastic bodies interact with articulated
   rigid chains and stacks.
8. **A7 - candidate generation and persistence:** Build broad-phase/narrow-phase
   row generation with persistent row IDs for warm-started lambda/stiffness,
   including CPU and GPU candidate paths.
9. **A8 - Hessian and stability policy:** Implement the quasi-Newton diagonal
   geometric-stiffness approximation, post-stabilization option, finite maximum
   bounds for conflicting constraints, and diagnostics for impossible row sets.
10. **A9 - CPU performance:** Add SoA layouts, task-parallel color sweeps,
    deterministic reductions, benchmark JSON, and direct comparisons against
    `avbd-demo2d`/`avbd-demo3d`.
11. **G1-G5 - GPU parity:** Port every row family, candidate-generation path,
    dual update, color update, and benchmark scene to the CUDA/private compute
    boundary; record same-GPU packets for the published RTX-4090 cases.
12. **D1-D6 - demos and experiments:** Reproduce every source demo, online demo,
    paper figure, table, parameter sweep, and supplemental/video scenario in
    DART `py-demos`, tests, benchmarks, and visual capture gates.

## Current Next Gap

After the narrow free-rigid fixed-joint/contact, motor, breakable fixed
point-joint public-World benchmark slices, the first non-empty `avbd-demo2d`
Ground, Motor, Hanging Rope, Fracture, Dynamic Friction, Static Friction,
Pyramid, Cards, Stack, Stack Ratio, Rod, Joint Grid, Rope, Heavy Rope, and Net source-row
ports,
and the first `avbd-demo3d` Ground, Dynamic Friction, Static Friction, Pyramid,
Rope, Heavy Rope, Stack, Stack Ratio, Bridge, and Breakable source-row ports,
the highest-risk missing paper
capability is still articulated
AVBD: the private extraction path now classifies multibody link
endpoints explicitly and has a first hard fixed/masked/breakable point-joint
bridge into the variational projection path, including narrow hard
bounded revolute/prismatic velocity-motor rows with revolute/prismatic
command-update coverage, private fixed-row reset, revolute/prismatic
break/reset re-engagement, and persistent
stiffness-ramped compliant fixed rows. Non-topology
multibody-link fixed/revolute/prismatic point-joint
entities can also generate hard private configs from simulation-entry current
poses, and public same-multibody/world-link articulated
fixed/revolute/prismatic/spherical facades now feed that extractor through
C++/dartpy with focused spherical linear-only pinned-anchor behavior including explicit link-link/world-link anchors, bounded
revolute/prismatic velocity-actuator, same-multibody/world-anchored
command-update, same-multibody/world-anchored public one-DOF motor break/skip,
same-multibody/world-anchored public one-DOF motor break/reset re-engagement,
selected direct same-multibody/world-link one-DOF break/skip/reset
non-cardinal basis checks plus direct private parent-endpoint one-DOF reset
re-engagement and child-/parent-endpoint command-update non-cardinal axis-basis
coverage plus direct private child-/parent-endpoint finite-effort non-cardinal
axis-basis coverage,
focused dartpy stepping coverage for same-multibody/world-link revolute and
prismatic explicit-anchor break/skip and reset/re-engagement cases with
post-reset endpoint/axis-shape assertions including non-cardinal
same-multibody pair and world-link revolute/prismatic reset regressions plus
C++/dartpy non-cardinal
off-origin finite-effort cap regressions spanning same-multibody/world-link
revolute and prismatic facades plus dartpy binary round-trip coverage for
all same-multibody/world-link fixed/spherical/revolute/prismatic design-mode
rebuilds, all same-multibody/world-link revolute/prismatic broken motor states,
and all same-multibody/world-link fixed/spherical broken state before reset
re-engagement,
movable-movable
same-multibody link-pair motor projection,
explicit local/world anchor projection, same-multibody fixed and world-fixed
break/reset, same-multibody/world-link spherical linear-row break/skip and
reset including focused dartpy spherical stepping coverage with endpoint-shape
assertions, same-multibody link-link and world-link
fixed/spherical save/load rebuilding of the private all-axis and linear rows
including dartpy design-mode rebuild evidence,
same-multibody/world-link
revolute/prismatic motor save/load rebuilding of the private hard rows and
free-axis motor row with selected non-cardinal axis-basis persistence and
restored same-multibody/world-link endpoint-shape assertions,
same-multibody/world-link
fixed/spherical/revolute/prismatic broken-state save/load/reset persistence
including explicit-anchor fixed and selected non-cardinal one-DOF motor rows
with focused C++ binary round-trips preserving velocity-actuator
command/effort-limit state plus dartpy binary round-trips preserving selected
same-multibody/world-link fixed/spherical and one-DOF design-mode rebuilds,
all same-multibody/world-link one-DOF, fixed, and spherical broken state and
selected direct break/skip/reset non-cardinal basis checks, direct private and current-pose movable-pair revolute
break/reset coverage, non-cardinal
current-pose revolute/prismatic motor-axis coverage over generated free-axis
bases, public same-multibody/world-anchored articulated revolute/prismatic
floating-endpoint plus selected off-origin-anchor facade non-cardinal motor-axis
coverage over generated AVBD free-axis bases, selected same-multibody/world-link
save/load and broken-state save/load/reset non-cardinal axis-basis
persistence, private current-pose fixed
movable-pair break/reset coverage that re-engages the captured anchor and
relative orientation, private current-pose prismatic movable-pair break/reset
coverage that re-enters with an updated velocity command, private current-pose
revolute/prismatic movable-pair finite-limit coverage that keeps generated
free-axis motors bounded, public same-multibody/world-link one-DOF
non-cardinal off-origin finite-limit coverage, public same-multibody movable-pair
revolute/prismatic non-cardinal motor-axis and finite-limit coverage that keeps
bounded facade motors from becoming unbounded relative-coordinate targets, and
public same-multibody movable-pair one-DOF broken-state binary save/load/reset
coverage that preserves endpoint/type/axis/actuator command and effort-limit
state and proves reset rebuilds the hard rows plus updated free-axis motor
motion across two movable links, private generated current-pose movable-pair
fixed all-axis broken-state binary save/load/reset coverage that preserves
endpoints and captured current-pose anchors before reset rebuilds both
generated linear and angular hard rows, private generated current-pose movable-pair
one-DOF broken-state binary save/load/reset coverage that preserves
endpoint/type/axis/actuator command, effort-limit state, and captured
current-pose anchors before reset rebuilds the generated hard rows plus updated
free-axis motor motion across two movable links, private generated current-pose
movable-pair spherical broken-state binary save/load/reset coverage that
preserves endpoints and captured current-pose anchors before reset rebuilds
only the generated linear rows while relative orientation remains free, direct
private world-link one-DOF broken-state binary save/load/reset coverage that
preserves private `AvbdRigidWorldPointJointConfig` anchors, bases, masks,
endpoint shape, stiffness fields, actuator command, and effort-limit state
before reset re-engages the hard rows plus updated free-axis motor motion, with
direct revolute/prismatic coverage now spanning both child-link and parent-link
endpoint polarity plus in-memory parent-endpoint reset re-engagement, direct
private child-/parent-endpoint command-update coverage including non-cardinal
axis-basis coverage, and direct private child-/parent-endpoint tiny
effort-limit coverage including non-cardinal axis-basis coverage, direct
private world-link fixed/spherical broken-state binary save/load/reset coverage
that preserves private anchors, masks, fixed child-link target orientation, and
spherical parent/child endpoint shape before reset re-engages all fixed rows or
only the spherical linear anchor rows, and
private current-pose
spherical movable-pair break/reset coverage that re-engages only the linear rows, public
free-rigid/articulated stiffness facade binary persistence plus direct
C++/dartpy validation of articulated stiffness defaults, finite setters, and
invalid setter rejection, C++/dartpy endpoint-ownership rejection coverage, and
world-anchor
coverage, but DART still lacks broader persistent motor lifecycle coverage
beyond the one-DOF, direct/private movable-pair lifecycle, and generated
current-pose plus public/private movable-pair tiny-limit and save/load/reset
checks,
broad fracture lifecycle coverage beyond the now-covered 2D Fracture and 3D
Breakable source-demo fixed-joint break/reset rows, and broader
public articulated World facade coverage beyond those
linear-only spherical/link-link/world-link entrypoints and fixed/spherical
save/load rebuilding.
The articulated point-joint extractor now also builds a per-multibody
link-index cache before scanning AVBD point-joint configs, removing per-joint
structure/link ownership scans from same-multibody and world-link private row
extraction; focused articulated C++ tests and `/32` articulated benchmark smoke
cover the changed hot path. Rigid AVBD point-joint and distance-spring
extraction now also carries the validated projectable-body metadata through the
extracted inputs, avoiding a second transform/mass/static lookup when the
pair-constraint appenders materialize those rows and reusing the
already-checked projectable transform, mass, and static tag when contact
snapshots materialize body state. The same metadata path also skips
static-static rigid point-joint and distance-spring pairs before input or row
construction.
DART now has a narrow
five-link 200:1 high mass-ratio articulated-chain smoke scene and dashboard row,
plus a focused C++ 50-link/50,000:1 finite/reset smoke and
`BM_AvbdPaperScaleHighRatioChainStep` dashboard row through configured
`World::step()` solve-budget fields with
[`avbd-paper-scale-high-ratio-chain-packet.json`](avbd-paper-scale-high-ratio-chain-packet.json)
visual/benchmark evidence, but it still cannot reproduce the paper's full
hard-constrained articulated chains, same-hardware numbers, or
cloth-plus-articulated-rigid coupling scenes.
The next implementation slice should either optimize the measured
`avbd_demo2d_ground`, `avbd_demo2d_motor`,
`avbd_demo2d_hanging_rope`, `avbd_demo2d_rod`,
`avbd_demo2d_joint_grid`, `avbd_demo2d_rope`, `avbd_demo2d_heavy_rope`,
`avbd_demo2d_net`, `avbd_demo3d_rope`, or `avbd_demo3d_heavy_rope` CPU gaps,
port another bounded source row from the
corpus matrix, or keep filling the private articulated AVBD state/extraction
bridge before claiming contact-stage or source-demo completeness.
The latest Fracture-focused cleanup replaces a duplicate rigid-contact-stage
prepare-time collision query with collision-shape-count constraint prewarm and
keeps live constrained-pair filtering aligned with the native solver; a
refreshed same-source Fracture packet now records DART about 1.20x faster than
the native source runner, closing only that narrow source-row CPU comparison.

## Required Experiment And Demo Corpus

Minimum corpus before any AVBD parity claim. The durable row-by-row tracking
surface is
[`avbd-demo-corpus.md`](avbd-demo-corpus.md); keep this section as the compact
scope summary and update the matrix when scenes move from missing to partial or
complete.

- 2D demo scenes: empty, ground, dynamic friction, static friction, pyramid,
  card tower, rope, heavy rope, hanging rope, spring, spring ratio, stack, stack
  ratio, rod, soft body, joint grid, net, motor, and fracture.
- 3D demo scenes: empty, ground, dynamic friction, static friction, pyramid,
  rope, heavy rope, spring, spring ratio, stack, stack ratio, soft body, bridge,
  and breakable.
- Paper figures: high-stiffness ratio chains, flag/pole attachment, card tower,
  50-body pendulum with 50,000:1 mass ratio, two-heavy-ball chain, box stack,
  friction coefficient comparison, chain mail impact, breakable wall, large
  cloth plus articulated-rigid-body coupling, and parameter sweeps for `beta`,
  `alpha`, `gamma`, and iteration count.
- Website/video headline scenes: rigid stacking/piling with friction and the
  large block-pile/sphere-smash GPU scenes.

Each promoted scene needs a correctness invariant, a benchmark profile, and a
headless visual artifact or py-demo smoke where visual behavior matters.

## Performance Targets

Reference targets are not complete until measured in DART benchmark JSON:

- Paper Table 1, RTX 4090 GPU:
  - Figure 1, 110,000 bodies: AVBD 4 iterations at 3.5 ms/frame.
  - Figure 3, 510,000 bodies: AVBD 3 iterations at 10.3 ms/frame.
  - Baseline rows for sequential impulse, XPBD, and VBD must be reproduced or
    documented with audited equivalent setups before claiming relative speedup.
- Paper Figure 14: 35,000 rigid bodies, 72,000 joints, cloth with 10,000
  vertices and 20,000 triangles, 10 AVBD iterations, 16 ms/frame including
  collision detection.
- Reference demo CPU baselines: DART CPU must beat native `avbd-demo2d` and
  `avbd-demo3d` on matched scene counts, timestep, iteration count, and compiler
  mode before a CPU win is claimed. The current `avbd_demo2d_ground` packet
  times the native static Ground row and, after skipping static-only contact
  queries, no-op rigid dynamics stages, clean frame-cache graph execution, and
  the clean no-work default step pipeline with a cheap scratch reset, records
  the DART public World row about 1.51x faster on this host, so that narrow CPU
  win is claimed. The current
  `avbd_demo2d_motor` packet times the native source Motor row and, after
  skipping contact queries for no-collision-geometry worlds in contact-stage
  prepare/execute, records the DART public World row about 6.18x slower on this
  host, so no CPU win is claimed. The
  current `avbd_demo2d_hanging_rope` packet times the native source Hanging
  Rope row and records the DART public World row about 7.84x slower on this
  host, so no CPU win is claimed. The current
  `avbd_demo2d_dynamic_friction` packet times the native source Dynamic Friction
  row and records the DART public World row about 1.83x faster on this host.
  The current `avbd_demo2d_static_friction` packet times the native source
  Static Friction row and records the DART public World row about 2.68x faster
  on this host. The current `avbd_demo2d_pyramid` packet times the native source
  Pyramid row and records the DART public World row about 9.84x faster than the
  10,000-step native source runner on this host. The current
  `avbd_demo2d_cards` packet times the native source Cards row and records the
  DART public World row about 5.38x slower on this host; later friction
  tangent-pair world-anchor reuse, row-state constraint-vector reuse, and
  shared-anchor reuse removed local contact-heavy duplicate work but have not
  refreshed this packet, so no CPU win is claimed. The current
  `avbd_demo2d_stack` packet times the native source Stack row and records the
  DART public World row about 2.17x faster on this
  host. The current `avbd_demo2d_stack_ratio` packet times the native source
  Stack Ratio row and records the DART public World row about 2.22x faster on
  this host. The current `avbd_demo2d_rod` packet times the native source Rod
  row and records the DART public World row about 9.58x slower on this host, so
  no CPU win is claimed. The current `avbd_demo2d_joint_grid` packet times the
  native source Joint Grid row, configures the source's 1152 diagonal
  ignore-collision pairs through DART's public per-pair filter, and records the
  DART public World row about 2.06x slower on this host after reusing per-body
  row-index scratch and caching snapshot body indices. Later row-layout and
  no-copy solve overhead cleanup, angular-row orientation-error reuse, and
  precomputed world-anchor reuse through distance-spring Hessian/Jacobian
  stamping, plus friction tangent-pair world-anchor and row-state
  constraint-vector reuse plus shared-anchor reuse, improved local profile
  slices but have not refreshed this packet, so no CPU win is claimed.
  The current `avbd_demo2d_rope` packet times the native source Rope row and
  records the DART public World row about 5.20x slower on this host, so no CPU
  win is claimed. The current `avbd_demo2d_heavy_rope` packet times the native
  source Heavy Rope row and records the DART public World row about 5.85x
  slower on this host after reusing per-body row-index scratch and caching
  snapshot body indices, so no CPU win is claimed. The
  current `avbd_demo3d_ground` packet times the native source Ground row and
  records the DART public World row about 1.11x faster on this host, the
  current `avbd_demo3d_dynamic_friction` packet times the native source Dynamic
  Friction row and records the DART public World row about 1.41x faster, the
  current `avbd_demo3d_static_friction` packet times the native source Static
  Friction row and records the DART public World row about 1.08x faster, the
  current `avbd_demo3d_pyramid` packet times the native source Pyramid row and
  records the DART public World row about 2.83x faster, the current
  `avbd_demo3d_rope` packet times the native source Rope row and records the
  DART public World row about 3.75x slower, the current
  `avbd_demo3d_heavy_rope` packet times the native source Heavy Rope row and
  records the DART public World row about 3.84x slower, the current
  `avbd_demo3d_stack` packet times the native source Stack row and records the
  DART public World row about 1.80x faster, the current
  `avbd_demo3d_stack_ratio` packet times the native source Stack Ratio row and
  records the DART public World row about 2.32x faster, the current
  `avbd_demo3d_soft_body` packet times the native source Soft Body row and
  records the DART public World row about 1.21x faster, the current
  `avbd_demo3d_bridge` packet times the native source Bridge row and records
  the DART public World row about 1.61x faster, and the current
  `avbd_demo3d_breakable` packet times the native source Breakable row and
  records the DART public World row about 1.42x faster on this host. No broad
  CPU win is claimed until the matched source corpus is broader and the
  remaining slower rows are closed.
- DART GPU claims must use the same GPU class when comparing to published
  paper numbers, and must record fallback hardware separately.

## DART-Specific Constraints

- Keep public selection and config DART-owned and backend-neutral; avoid
  exposing reference project names, row-storage internals, ECS storage, CUDA
  types, or solver registries as public API.
- Do not vendor or link the reference demo repositories at runtime.
- Use existing DART foundations first: experimental `World`, deformable VBD
  kernels, deformable/rigid contact kernels, multibody dynamics, compute
  resource metadata, CUDA private boundary, benchmark JSON, and py-demos.
- Because DART 7 is an API-breaking clean-up release and this solver family is
  experimental, prefer long-term scalable API and pipeline refactors over
  backward-compatible compromises.
- Every slice must keep `pixi run lint`, focused build/tests, relevant
  benchmark or visual evidence, and `check-api-boundaries` green before it is
  described as complete.
