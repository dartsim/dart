# Evidence: DART 6.20 Examples Inventory

Collected 2026-07-04 from release-6.20 by recon agent. Full per-example table
lives in the recon transcript; this file keeps the decision-relevant facts.

## Shape of the catalog

- 37 buildable C++ examples (39 dirs − `rerun` orphan stub with no sources −
  octomap-gated `point_cloud` counted when HAVE_OCTOMAP). Zero GLUT usage; all
  GUI examples use `dart::gui::osg` (10 also build ImGui panels).
- 11 Python examples (dartpy), viewer-based except `hello_world` (console).
- Examples are NOT built by default (`add_subdirectory(examples
  EXCLUDE_FROM_ALL)` + aggregate `examples` target on non-MSVC), NOT
  installed, and each doubles as a standalone `find_package(DART ... CONFIG)`
  consumer sample via `dart_build_example_in_source`.

## Port workload (controller-bearing WorldNode subclasses)

Real logic to carry into scene factories (pre/post-step callbacks):
AtlasSimbiconWorldNode (SIMBICON FSM, 3.5k LOC multi-file), biped_stand
balance, JointConstraintsWorld (SPD), OperationalSpaceControlWorld,
HybridDynamicsWorld, VehicleWorld, ContactInverseDynamicsWorldNode (IK +
contact ID), TeleoperationWorld (atlas/hubo puppet IK), SsikWorldNode,
WamWorld (ikfast), HumanJointLimitsWorldNode (custom joint-limit
constraints), HeightmapWorld, DeactivationWorldNode (sleeping),
RigidChain/RigidCubes/RigidLoop nodes. Pure-render/trivial: empty, imgui,
box_stacking, rigid_shapes, simulation_event_handler, boxes, hello_world,
add_delete_skels.

## Cleanup constraints (Phase 5 gates)

- `contact_benchmark`: **CI-load-bearing** — `tests/benchmark/integration/
  CMakeLists.txt:68` pulls its sources; `performance_dashboard_dart6.yml`
  path-triggers on `examples/contact_benchmark/**`. Keep standalone (or move
  with CI updates in the same PR).
- `speed_test`: console benchmark; keep standalone or fold into benchmarks.
- `cylindrical_constraint`: console-only; not a GUI scene as-is (port adds a
  viewer or it stays console).
- `point_cloud` scene must stay behind `HAVE_OCTOMAP`; `wam_ikfast` carries
  ikfast generated code; `fetch` needs MJCF assets — all foldable with
  guards.
- `hello_world` (C++ & Python): keep standalone as the canonical minimal
  example and installed-DART consumer sample.
- `rerun`: orphan stub (CMakeLists only, unregistered) — delete.
- No example-smoke CI exists today; the demos cycle smoke will be the first.

## Python examples → py-demos scenes

hello_world_gui, rigid_cubes, rigid_chain, rigid_loop, drag_and_drop,
biped_stand, operational_space_control, atlas_puppet, contacts_pointcloud,
ssik_analytical_ik (ImGui-based — proves dartpy ImGui widget bindings work).
