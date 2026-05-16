# DART Native Collision — Test & Benchmark Coverage Matrix

This file is the authoritative tracker for what DART's native collision
test + benchmark suite covers today, what it should cover to be the
**superset** of every comparable physics-engine collision test in the
wild, and where the gaps are. The goal is not "match upstream X" — it
is "be the strict superset so DART native is provably feature-complete
AND demonstrably faster than every alternative."

Categories below are sourced from a cross-survey of major open-source
physics collision libraries. They are NOT attributed per-engine because
the goal is for DART to own this taxonomy as its own quality bar, not
to chase any one upstream.

## How To Read This File

- **Codename** — stable kebab-case ID. Tests / benchmarks reference
  these in their names (e.g. `TEST(BoxBox, boxbox_edge_edge)`) so a
  grep can find coverage immediately.
- **Status** — `DONE` (test + benchmark coverage exists), `PARTIAL`
  (test exists but no benchmark, or vice versa), `GAP` (not covered),
  `DEFERRED` (explicitly outside the current feature-level pass).
- **Bar** — what "covered" means for that row. A `DONE` row must meet
  its bar; a `PARTIAL` row must say what's missing.
- **Source** — file path(s) in this repo if covered.

When Codex changes the test/benchmark surface, update this file in
the same commit.

## TDD Workflow

Per the durable `Test-Driven Development` policy on this branch, every
new collision feature or fix lands in this order:

1. Add the row to this matrix as `GAP` with the proposed codename.
2. Write the failing test under that codename.
3. Confirm it fails for the right reason.
4. Implement the fix in the smallest scope.
5. Confirm the test passes.
6. Flip the row to `DONE` and re-record the benchmark JSON if applicable.

Round 7's box-box face-clipping work is the first slice to use this
workflow end-to-end (see SUPERVISOR.md Round 7).

## 1. Pair-Wise Narrow-Phase (collide + distance)

Bar per row: at least one test asserts correct contact count, depth,
and normal direction for that pair in a non-degenerate configuration
AND at least one test asserts the determinism + pair-order symmetry
contract (A×B and B×A produce contacts whose positions match and
normals flip).

| Codename                             | Status | Source                                                              | Notes                                                         |
| ------------------------------------ | ------ | ------------------------------------------------------------------- | ------------------------------------------------------------- |
| `sphere_sphere_basic`                | DONE   | `test_sphere_sphere.cpp`, `test_narrow_phase.cpp`                   | Multi-config sweep                                            |
| `sphere_sphere_zero_radius`          | DONE   | `test_sphere_sphere.cpp::ZeroRadius`                                | Degenerate radius                                             |
| `sphere_sphere_concentric`           | DONE   | `test_sphere_sphere.cpp`                                            | Coincident centers                                            |
| `sphere_sphere_determinism`          | DONE   | `test_sphere_sphere.cpp`                                            |                                                               |
| `sphere_sphere_pair_order`           | DONE   | `test_sphere_sphere.cpp`                                            |                                                               |
| `sphere_box_face`                    | DONE   | `test_sphere_box.cpp`                                               |                                                               |
| `sphere_box_corner`                  | DONE   | `test_sphere_box.cpp`                                               |                                                               |
| `sphere_box_edge`                    | DONE   | `test_sphere_box.cpp`                                               |                                                               |
| `sphere_box_center_inside`           | DONE   | `test_sphere_box.cpp`                                               | Deep penetration                                              |
| `sphere_box_determinism`             | DONE   | `test_sphere_box.cpp`                                               |                                                               |
| `sphere_box_pair_order`              | DONE   | `test_sphere_box.cpp`                                               |                                                               |
| `sphere_capsule_middle`              | DONE   | `test_capsule_capsule.cpp`                                          |                                                               |
| `sphere_capsule_caps`                | DONE   | `test_capsule_capsule.cpp`                                          | Both endpoints                                                |
| `sphere_cylinder_side`               | DONE   | `test_cylinder.cpp`                                                 |                                                               |
| `sphere_cylinder_top`                | DONE   | `test_cylinder.cpp`                                                 |                                                               |
| `sphere_cylinder_top_edge`           | DONE   | `test_cylinder.cpp`                                                 | Edge between cap + side                                       |
| `sphere_plane_*`                     | DONE   | `test_plane.cpp`                                                    |                                                               |
| `sphere_mesh_basic`                  | DONE   | `test_mesh_mesh.cpp`, `test_dart_collision_detector.cpp`            |                                                               |
| `sphere_mesh_pair_order`             | DONE   | `test_dart_collision_detector.cpp`                                  | Renamed in Round 5                                            |
| `sphere_convex_intersecting`         | DONE   | `test_convex.cpp`                                                   |                                                               |
| `sphere_sdf_distance`                | DONE   | `test_sdf_compare.cpp`                                              |                                                               |
| `sphere_compound_basic`              | DONE   | `test_compound.cpp`                                                 |                                                               |
| `boxbox_basic`                       | DONE   | `test_box_box.cpp`                                                  |                                                               |
| `boxbox_separated_axis_xyz`          | DONE   | `test_box_box.cpp`                                                  |                                                               |
| `boxbox_touching`                    | DONE   | `test_box_box.cpp`                                                  |                                                               |
| `boxbox_overlapping_face_patch`      | DONE   | `test_box_box.cpp::RotatedBoxOnFlatGroundEmitsFacePatch`            | Rotated near-face patch emits >=3 clipped contacts            |
| `boxbox_edge_edge`                   | DONE   | `test_box_box.cpp::EdgeEdgeTiltedContactEmitsSingleSupportPoint`    | Edge-edge tilted contact — Round 7                            |
| `boxbox_face_vertex`                 | DONE   | `test_box_box.cpp::FaceVertexContactAgainstGroundFace`              | One vertex against opposing face — Round 7                    |
| `boxbox_rotated_on_ground`           | DONE   | `test_box_box.cpp` + `test_world.cpp`                               | Static contact patch + dynamic rest checks                    |
| `boxbox_rotated_settles_on_face`     | DONE   | `test_world.cpp::DefaultNativeRotatedBoxSettlesOnFace`              | Dynamic rest condition — Round 7 acceptance #2                |
| `boxbox_15s_no_tunneling`            | DONE   | `test_world.cpp::DefaultNativeRotatedBoxStaysOnGround15s`           | Long-horizon stability — Round 7 acceptance #3                |
| `boxbox_sat_axis_no_flicker`         | DONE   | `test_box_box.cpp::SatAxisStableForNearFaceBoxGroundPerturbations`  | SAT axis stability under perturbation — Round 7 acceptance #5 |
| `boxbox_determinism`                 | DONE   | `test_box_box.cpp:411`                                              |                                                               |
| `boxbox_pair_order_swap`             | DONE   | `test_box_box.cpp:427`                                              | Round 3                                                       |
| `boxbox_distance`                    | DONE   | `test_distance_core.cpp`                                            |                                                               |
| `box_capsule_*`                      | DONE   | `test_capsule_capsule.cpp`                                          | Face / corner / penetrating / rotated                         |
| `box_cylinder_*`                     | DONE   | `test_cylinder.cpp`                                                 | Side / face / on-top / axial cap patch                        |
| `box_plane_*`                        | DONE   | `test_plane.cpp`                                                    |                                                               |
| `box_mesh_colliding`                 | DONE   | `test_mesh_mesh.cpp`                                                |                                                               |
| `box_mesh_large_flat_patch_capped`   | DONE   | `test_mesh_mesh.cpp`                                                |                                                               |
| `box_mesh_atlas_foot_ground`         | DONE   | `test_world.cpp::AtlasSimbiconFeetContactGroundWithNativeCollision` | Real Atlas foot mesh vs ground box regression                 |
| `box_convex_intersecting`            | DONE   | `test_convex.cpp`                                                   |                                                               |
| `box_compound_basic`                 | DONE   | `test_compound.cpp`                                                 |                                                               |
| `capsule_capsule_parallel_overlap`   | DONE   | `test_capsule_capsule.cpp`                                          |                                                               |
| `capsule_capsule_end_to_end`         | DONE   | `test_capsule_capsule.cpp`                                          |                                                               |
| `capsule_capsule_perpendicular`      | DONE   | `test_capsule_capsule.cpp`                                          |                                                               |
| `capsule_capsule_distance`           | DONE   | `test_capsule_capsule.cpp`                                          |                                                               |
| `capsule_plane_standing`             | DONE   | `test_plane.cpp`                                                    |                                                               |
| `capsule_plane_lying`                | DONE   | `test_plane.cpp`                                                    |                                                               |
| `capsule_cylinder_*`                 | DONE   | `test_cylinder.cpp`                                                 | Parallel + capsule-on-top                                     |
| `capsule_mesh`                       | DONE   | `test_mesh_mesh.cpp::CapsuleVsMeshPairOrder`                        |                                                               |
| `capsule_convex`                     | DONE   | `test_convex.cpp::CapsuleConvexPairOrder`                           |                                                               |
| `capsule_sdf`                        | DONE   | `test_sdf_compare.cpp`                                              |                                                               |
| `capsule_compound`                   | DONE   | `test_compound.cpp::CapsulePairOrder`                               |                                                               |
| `cylinder_cylinder_parallel_overlap` | DONE   | `test_cylinder.cpp`                                                 |                                                               |
| `cylinder_cylinder_stacked`          | DONE   | `test_cylinder.cpp`                                                 |                                                               |
| `cylinder_cylinder_perpendicular`    | DONE   | `test_cylinder.cpp`                                                 |                                                               |
| `cylinder_plane_*`                   | DONE   | `test_cylinder.cpp`                                                 | Standing / lying / tilted / distance                          |
| `cylinder_mesh`                      | DONE   | `test_mesh_mesh.cpp::CylinderVsMeshPairOrder`                       |                                                               |
| `cylinder_convex`                    | DONE   | `test_convex.cpp::CylinderConvexPairOrder`                          |                                                               |
| `cylinder_sdf`                       | DONE   | `test_sdf_compare.cpp::CylinderSdfPairOrder`                        | Distance support                                              |
| `cylinder_compound`                  | DONE   | `test_compound.cpp::CylinderPairOrder`                              |                                                               |
| `plane_mesh_penetrating`             | DONE   | `test_plane.cpp`                                                    |                                                               |
| `plane_mesh_broadphase_path`         | DONE   | `test_dart_collision_detector.cpp`                                  |                                                               |
| `plane_mesh_pair_order`              | DONE   | `test_narrow_phase.cpp`                                             |                                                               |
| `plane_convex`                       | DONE   | `test_convex.cpp::PlaneConvexPairOrder`                             |                                                               |
| `plane_compound`                     | DONE   | `test_compound.cpp::PlanePairOrder`                                 |                                                               |
| `mesh_mesh_basic`                    | DONE   | `test_mesh_mesh.cpp`                                                |                                                               |
| `mesh_mesh_separated`                | DONE   | `test_mesh_mesh.cpp`                                                |                                                               |
| `mesh_mesh_single_triangle_vs_large` | DONE   | `test_mesh_mesh.cpp`                                                |                                                               |
| `mesh_mesh_contact_plane`            | DONE   | `test_mesh_contact_regression.cpp`                                  |                                                               |
| `mesh_convex`                        | DONE   | `test_convex.cpp::MeshConvexPairOrder`                              | Routes through GJK                                            |
| `mesh_sdf`                           | DONE   | `test_sdf_compare.cpp::MeshSdfPairOrder`                            | Distance support                                              |
| `mesh_compound`                      | DONE   | `test_compound.cpp::MeshPairOrder`                                  |                                                               |
| `convex_convex_intersecting`         | DONE   | `test_convex.cpp`                                                   |                                                               |
| `convex_convex_separated`            | DONE   | `test_convex.cpp`                                                   |                                                               |
| `convex_convex_rotated`              | DONE   | `test_convex.cpp`                                                   |                                                               |
| `convex_sdf`                         | DONE   | `test_sdf_compare.cpp::ConvexSdfPairOrder`                          | Distance support                                              |
| `convex_compound`                    | DONE   | `test_compound.cpp::ConvexPairOrder`                                |                                                               |
| `sdf_sdf`                            | DONE   | `test_sdf_compare.cpp::SdfSdfPairOrder`                             | Distance support                                              |
| `sdf_compound`                       | DONE   | `test_sdf_compare.cpp::CompoundSdfPairOrder`                        | Distance support                                              |
| `compound_compound_basic`            | DONE   | `test_compound.cpp`                                                 |                                                               |
| `compound_compound_separated`        | DONE   | `test_compound.cpp`                                                 |                                                               |

**Adapter-routed shapes** (DART has these dynamics shapes but the
native taxonomy does not have a dedicated `ShapeType` for them; they
adapt to convex/mesh/compound):

| Codename              | Status | Source                             | Notes |
| --------------------- | ------ | ---------------------------------- | ----- |
| `adapter_cone`        | DONE   | `test_collision_backend.cpp`       |       |
| `adapter_ellipsoid`   | DONE   | `test_collision_backend.cpp`       |       |
| `adapter_heightmap`   | DONE   | `test_collision_backend.cpp`       |       |
| `adapter_multisphere` | DONE   | `test_collision_backend.cpp`       |       |
| `adapter_voxelgrid`   | DONE   | `test_collision_backend.cpp`       |       |
| `adapter_pyramid`     | DONE   | `test_dart_collision_detector.cpp` |       |

## 2. Algorithm-Level

Bar per row: at least one test exercises the algorithm in isolation
(not through `NarrowPhase::collide`) AND covers degenerate inputs.

| Codename                              | Status   | Source                                                                       | Notes                                                                                  |
| ------------------------------------- | -------- | ---------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `gjk_distance_query`                  | DONE     | `test_gjk.cpp`                                                               |                                                                                        |
| `gjk_intersection_query`              | DONE     | `test_gjk.cpp`                                                               |                                                                                        |
| `gjk_warm_start_cache`                | DONE     | `test_gjk.cpp::WarmStartSimplexReuseMatchesColdQuery`                        | Warm-start via simplex reuse                                                           |
| `gjk_degenerate_segments`             | DONE     | `test_gjk_degenerate.cpp`                                                    |                                                                                        |
| `gjk_libccd_parity`                   | DONE     | `test_libccd_algorithms.cpp`                                                 |                                                                                        |
| `epa_penetration_depth`               | DONE     | `test_gjk.cpp::BoxBoxPenetrationDepthAnalytic`                               | Standalone analytic depth check                                                        |
| `epa_libccd_parity`                   | DONE     | `test_libccd_algorithms.cpp`                                                 |                                                                                        |
| `epa_signed_distance`                 | DONE     | `test_gjk.cpp::BoxBoxSignedDistanceAnalytic`                                 | Negative distance via EPA                                                              |
| `mpr_penetration_depth`               | DONE     | `test_gjk.cpp::SphereSpherePenetrationDepthAnalytic`                         | Standalone analytic depth check                                                        |
| `mpr_libccd_parity`                   | DONE     | `test_libccd_algorithms.cpp`                                                 |                                                                                        |
| `sat_box_axes`                        | DONE     | `test_box_box.cpp::SatAxisStableForNearFaceBoxGroundPerturbations`           | Calls `computeBoxBoxSat` directly                                                      |
| `sat_axis_no_flicker`                 | DONE     | `test_box_box.cpp::SatAxisStableForNearFaceBoxGroundPerturbations`           | Round 7 acceptance #5                                                                  |
| `sat_axis_face_bias_tie`              | DONE     | `test_box_box.cpp::SatAxisFaceBiasTiePrefersFace`                            | Round 7 RC3 fix verification                                                           |
| `sat_skip_degenerate_cross_axes`      | DONE     | `test_box_box.cpp::SatSkipsDegenerateCrossAxes`                              | Round 7 RC2 fix verification                                                           |
| `bvh_build_from_triangles`            | DONE     | `test_shapes.cpp::MeshShape.BvhLargeMesh`, `BvhDegenerateTriangles`          | Build invariants and degenerate triangle coverage                                      |
| `bvh_build_from_points`               | DEFERRED | `test_dart_collision_detector.cpp::PointCloudShapeIsExplicitlyNonCollidable` | Point clouds are explicit non-collidable adapter shapes in this feature pass           |
| `bvh_traversal_hit`                   | DONE     | `test_mesh_mesh.cpp::RaycastMesh.BvhTraversalHit`                            |                                                                                        |
| `bvh_refit_after_transform`           | DONE     | `test_mesh_mesh.cpp::MeshMesh.BvhTraversalUsesCurrentTransform`              | Local BVH traversal follows the current query transform                                |
| `bvh_traversal_with_front_list`       | DEFERRED | —                                                                            | Warm-start incremental BVH belongs to the performance wave                             |
| `bvh_overlap_aabb_only`               | DONE     | `test_aabb_tree.cpp`                                                         |                                                                                        |
| `bvh_overlap_obb`                     | DEFERRED | —                                                                            | DART native has only AABB BVH today; OBB BVH is a future-wave decision                 |
| `bvh_overlap_rss`                     | DEFERRED | —                                                                            | Same                                                                                   |
| `bvh_kdop`                            | DEFERRED | —                                                                            | Decide if k-DOP is in DART's native scope                                              |
| `persistent_manifold_create`          | DONE     | `test_collision_backend.cpp::PersistentManifoldCache*`                       |                                                                                        |
| `persistent_manifold_pair_key`        | DONE     | `test_collision_backend.cpp`                                                 | Pair-key symmetry                                                                      |
| `persistent_manifold_warm_start`      | DONE     | `test_collision_backend.cpp`                                                 |                                                                                        |
| `persistent_manifold_refresh_drift`   | DONE     | `test_collision_backend.cpp`                                                 |                                                                                        |
| `persistent_manifold_threshold_break` | DONE     | `test_collision_backend.cpp`                                                 |                                                                                        |
| `persistent_manifold_reduction`       | DONE     | `test_collision_backend.cpp`                                                 |                                                                                        |
| `ccd_sphere_cast_*`                   | DONE     | `test_ccd.cpp`                                                               | Sphere/Box/Plane/Cylinder/Convex/Mesh targets                                          |
| `ccd_capsule_cast_*`                  | DONE     | `test_ccd.cpp`                                                               | Capsule/Box/Convex/Mesh targets                                                        |
| `ccd_conservative_advancement_convex` | DONE     | `test_ccd.cpp::ConservativeAdvancement*`, `bm_ccd.cpp`                       | Unit and benchmark coverage                                                            |
| `ccd_spline_motion`                   | DEFERRED | —                                                                            | Decide if non-linear motion is in scope                                                |
| `ccd_screw_motion`                    | DEFERRED | —                                                                            | Same                                                                                   |
| `ccd_bilateral_advancement`           | DEFERRED | —                                                                            | Two-body conservative advancement                                                      |
| `broadphase_brute_force`              | DONE     | `test_brute_force.cpp`                                                       |                                                                                        |
| `broadphase_aabb_tree`                | DONE     | `test_aabb_tree.cpp`                                                         |                                                                                        |
| `broadphase_aabb_tree_bulk_build`     | DONE     | `test_aabb_tree.cpp`                                                         |                                                                                        |
| `broadphase_sap`                      | DONE     | `test_sweep_and_prune.cpp`                                                   |                                                                                        |
| `broadphase_spatial_hash`             | DONE     | `test_spatial_hash.cpp`                                                      |                                                                                        |
| `broadphase_parity_brute_force`       | DONE     | `test_aabb_tree.cpp` / `test_sap.cpp` / `test_spatial_hash.cpp`              | Cross-validation against brute force                                                   |
| `broadphase_octree`                   | DEFERRED | `test_collision_backend.cpp::VoxelGridShapeAdapter`                          | VoxelGrid adapter coverage exists; native octree broadphase is a future scope decision |
| `contact_filter_composite`            | DONE     | `test_collision_filter_core.cpp`                                             |                                                                                        |
| `contact_filter_bodynode`             | DONE     | `test_collision_filter_core.cpp`                                             |                                                                                        |
| `distance_filter`                     | DONE     | `test_distance_filter.cpp`                                                   |                                                                                        |
| `sdf_dense_field`                     | DONE     | `test_sdf_compare.cpp`                                                       |                                                                                        |
| `sdf_esdf_voxblox`                    | DONE     | `test_sdf_compare.cpp`                                                       |                                                                                        |
| `auto_diff_narrow_phase`              | DEFERRED | —                                                                            | Run pair tests under autodiff scalar (future)                                          |
| `float_double_parity`                 | DEFERRED | —                                                                            | DART is double-only today; in-scope if we add `float` instantiation                    |

## 3. Stress / Regression / Determinism

Bar: scene-level test that runs > 100 steps OR > 100 collision pairs
and asserts a stability invariant (no tunneling, no false contacts,
deterministic across threads, etc.).

| Codename                              | Status | Source                                                                                     | Notes                                                               |
| ------------------------------------- | ------ | ------------------------------------------------------------------------------------------ | ------------------------------------------------------------------- |
| `rotated_box_rests_on_ground`         | DONE   | `test_world.cpp::DefaultNativeRotatedBoxRestsOnGround`                                     | Extended by face-rest and long-horizon checks                       |
| `rotated_box_settles_on_face_2s`      | DONE   | `test_world.cpp::DefaultNativeRotatedBoxSettlesOnFace`                                     | Round 7 acceptance #2                                               |
| `rotated_box_15s_no_tunneling`        | DONE   | `test_world.cpp::DefaultNativeRotatedBoxStaysOnGround15s`                                  | Round 7 acceptance #3 — the load-bearing stability test             |
| `hello_world_box_no_tunneling`        | DONE   | `test_world.cpp::DefaultNativeHelloWorldBoxDoesNotTunnel`                                  | Exact example-style free-joint and shape-node setup                 |
| `atlas_simbicon_controller_no_tunnel` | DONE   | `test_world.cpp::AtlasSimbiconControllerFeetStayAboveGroundWithNativeCollision`            | Example controller loop keeps feet above ground with native contact |
| `stacked_boxes_n10`                   | DONE   | `test_world.cpp::DefaultNativeTenBoxStackDoesNotTunnel`                                    | Simple vertical stack stability                                     |
| `stacked_boxes_n100`                  | DONE   | `test_world.cpp::DefaultNativeHundredBoxWallDoesNotTunnel`                                 | 100-box wall stability                                              |
| `mixed_primitive_stack`               | DONE   | `test_world.cpp::DefaultNativeMixedPrimitiveStackDoesNotTunnel`, `bm_mixed_primitives.cpp` | Unit stability and benchmark coverage                               |
| `ragdoll_capsule_pile`                | DONE   | `test_world.cpp::DefaultNativeCapsulePileDoesNotTunnel`                                    | Multiple capsule bodies                                             |
| `convex_static_mesh_landscape`        | DONE   | `test_world.cpp::DefaultNativeConvexBodiesSettleOnStaticMeshLandscape`                     | BVH-driven convex-concave under load                                |
| `voronoi_fracture_settle`             | DONE   | `test_world.cpp::DefaultNativeConvexFragmentsSettleOnMeshLandscape`                        | Many small convex hulls                                             |
| `parallel_narrow_phase_determinism`   | DONE   | `test_parallel_determinism.cpp`                                                            |                                                                     |
| `collide_all_ordering_repeatable`     | DONE   | `test_collision_world.cpp`                                                                 |                                                                     |
| `raycast_all_ordering_repeatable`     | DONE   | `test_collision_world.cpp`                                                                 |                                                                     |
| `persistent_scene_dirty_shape`        | DONE   | `test_dart_collision_detector.cpp`                                                         |                                                                     |
| `persistent_scene_dirty_mesh`         | DONE   | `test_dart_collision_detector.cpp`                                                         |                                                                     |
| `persistent_scene_dirty_heightmap`    | DONE   | `test_dart_collision_detector.cpp`                                                         |                                                                     |
| `persistent_scene_dirty_voxelgrid`    | DONE   | `test_dart_collision_detector.cpp`                                                         |                                                                     |
| `mesh_contact_plane_stability`        | DONE   | `test_mesh_contact_regression.cpp`                                                         |                                                                     |
| `gz_issue1184_accuracy`               | DONE   | `test_collision_accuracy.cpp`                                                              |                                                                     |
| `gz_issue1231_no_false_contacts`      | DONE   | `test_no_false_contacts.cpp`                                                               |                                                                     |
| `gz_issue1624_contact_grouping`       | DONE   | `test_contact_grouping.cpp`                                                                |                                                                     |
| `gz_issue1654_capsule_ground`         | DONE   | `test_capsule_ground_contact.cpp`                                                          |                                                                     |
| `gz_issue867_box_stack`               | DONE   | `test_bullet_box_stack.cpp`                                                                |                                                                     |
| `gz_issue895_self_collision`          | DONE   | `test_self_collision_filtering.cpp`                                                        |                                                                     |
| `cross_backend_consistency`           | DONE   | `test_native_backend_consistency.cpp`                                                      | Gated on reference engines                                          |
| `cross_backend_contact_count_parity`  | DONE   | `test_box_ground_contact_parity.cpp`                                                       | Reference-enabled parity test; match Bullet contact count within ±1 |
| `thin_box_no_tunneling`               | DONE   | `test_world.cpp::DefaultNativeThinBoxDoesNotTunnel`                                        | Slender feature stress                                              |
| `slender_capsule_no_tunneling`        | DONE   | `test_world.cpp::DefaultNativeSlenderCapsuleDoesNotTunnel`                                 | Capsule line-support stress                                         |
| `coincident_vertex_no_crash`          | DONE   | `test_narrow_phase.cpp::CoincidentVertexMeshNoCrashAcrossBatch`                            | Degenerate mesh batch regression                                    |
| `zero_extent_shape_no_crash`          | DONE   | `test_narrow_phase.cpp::ZeroExtentShapesNoCrashAcrossBatch`                                | Zero-size primitive batch regression                                |
| `huge_magnitude_fp_stability`         | DONE   | `test_narrow_phase.cpp::HugeMagnitudeBoxesRemainFiniteAcrossBatch`                         | 1e6 m boxes                                                         |
| `tiny_magnitude_fp_stability`         | DONE   | `test_narrow_phase.cpp::TinyMagnitudeSpheresRemainFiniteAcrossBatch`                       | 1e-6 m spheres                                                      |

## 4. Benchmarks

Bar: every pair / algorithm / scene listed in §1-3 should have a
benchmark variant (or be intentionally marked "no benchmark — pure
correctness"). Cross-engine comparisons (vs FCL / Bullet / ODE)
should use the apples-to-apples adapter path (see Q4 — `08-pair-coverage.md`
Query-parity note).

### Per-pair narrow-phase

| Codename                              | Status | JSON output                                 | Notes                                                                                                                                                                              |
| ------------------------------------- | ------ | ------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `bench_narrow_phase_per_pair_raw`     | DONE   | `native_collision_raw_narrow_phase.json`    | Native raw vs reference adapter — see Q4 parity gap                                                                                                                                |
| `bench_narrow_phase_per_pair_adapter` | DONE   | `collision_check_narrow_adapter.json`       | Native/reference through public detector adapter path                                                                                                                              |
| `bench_narrow_phase_per_pair_ref_raw` | DONE   | `collision_check_narrow_raw_reference.json` | SphereSphere/BoxBox/SphereBox fixed raw rows plus primitive edge-case raw rows for SphereSphere, BoxBox, CapsuleCapsule, SphereBox, CapsuleSphere, and CapsuleBox at scale index 1 |

### Distance + raycast + CCD

| Codename                         | Status   | JSON output                                                | Notes                                        |
| -------------------------------- | -------- | ---------------------------------------------------------- | -------------------------------------------- |
| `bench_distance_per_pair`        | DONE     | `collision_check_distance.json`                            | Sphere/Box/Capsule + EdgeCases               |
| `bench_distance_edge_cases`      | DONE     | `collision_check_distance.json`                            |                                              |
| `bench_distance_scale_sweep`     | DEFERRED | —                                                          | Performance-wave scale sweep: 1e-3 / 1 / 1e3 |
| `bench_raycast_per_primitive`    | DONE     | `collision_check_raycast.json`                             | Native vs Bullet only; FCL/ODE missing       |
| `bench_raycast_batch`            | DONE     | `collision_check_raycast_batch.json`                       | Native vs Bullet                             |
| `bench_sphere_cast_per_target`   | DONE     | `bm_ccd.cpp` (no JSON name; gated on `bm-collision-check`) |                                              |
| `bench_capsule_cast_per_target`  | DONE     | `bm_ccd.cpp`                                               |                                              |
| `bench_conservative_advancement` | DONE     | `bm_ccd.cpp`                                               |                                              |

### Broadphase + scene

| Codename                                   | Status   | JSON output         | Notes                                                         |
| ------------------------------------------ | -------- | ------------------- | ------------------------------------------------------------- |
| `bench_broadphase_per_impl_add`            | DONE     | `bm_broadphase.cpp` | BruteForce / AabbTree / SAP / SpatialHash                     |
| `bench_broadphase_per_impl_update`         | DONE     | `bm_broadphase.cpp` |                                                               |
| `bench_broadphase_per_impl_query_overlap`  | DONE     | `bm_broadphase.cpp` |                                                               |
| `bench_broadphase_per_impl_query_pairs`    | DONE     | `bm_broadphase.cpp` |                                                               |
| `bench_broadphase_scaling_n100_n1000`      | DEFERRED | —                   | Performance-wave per-impl scaling sweep at N=100, 1000, 10000 |
| `bench_dbvt_simd_intersect_select_merge`   | DEFERRED | —                   | If DART adds SIMD BVH ops                                     |
| `bench_persistent_manifold_cache_hit_rate` | DEFERRED | —                   | Performance-wave per-frame cache-hit ratio under perturbation |

### Scenarios

| Codename                             | Status   | JSON output                    | Notes                                                   |
| ------------------------------------ | -------- | ------------------------------ | ------------------------------------------------------- |
| `bench_scenario_mixed_primitives`    | DONE     | `collision_check_mixed.json`   | Dense + Sparse × 100/1000                               |
| `bench_scenario_mesh_heavy`          | DONE     | `collision_check_mesh.json`    | Native vs Bullet/FCL                                    |
| `bench_scenario_stacked_boxes`       | DEFERRED | —                              | Performance-wave vertical stack of N=10..1000 boxes     |
| `bench_scenario_pyramid_wall_tower`  | DEFERRED | —                              | Performance-wave mixed-stack settling                   |
| `bench_scenario_ragdoll_pile`        | DEFERRED | —                              | Performance-wave capsule-capsule + capsule-box at scale |
| `bench_scenario_convex_vs_landscape` | DEFERRED | —                              | Performance-wave BVH-driven convex-concave under load   |
| `bench_scenario_pipeline_breakdown`  | DONE     | `bm_pipeline_breakdown.cpp`    | Per-stage timing                                        |
| `bench_scenario_dart_adapter`        | DONE     | `collision_check_adapter.json` | Public adapter persistent scene                         |

### Algorithm-level micro-benchmarks

| Codename              | Status   | Notes                                      |
| --------------------- | -------- | ------------------------------------------ |
| `bench_gjk_per_pair`  | DONE     | `bm_libccd.cpp` covers SphereSphere/BoxBox |
| `bench_epa_per_pair`  | DONE     | `bm_libccd.cpp`                            |
| `bench_mpr_per_pair`  | DONE     | `bm_libccd.cpp`                            |
| `bench_bvh_build`     | DEFERRED | —                                          |
| `bench_bvh_traversal` | DEFERRED | —                                          |

### Batch + SIMD (Round 9 architectural direction)

Bar per row: a batch entry function exists for the pair, plus a
determinism test asserting batch results bit-match single-pair loop,
plus a benchmark sweeping batch size N=1/10/100/1000.

| Codename                                        | Status   | Notes                                                                                                                                                           |
| ----------------------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `boxbox_batch_determinism_vs_single`            | DONE     | `test_box_box.cpp::boxbox_batch_determinism_vs_single`                                                                                                          |
| `boxbox_batch_api_surface`                      | DONE     | `box_box.hpp` / `box_box.cpp`: `BoxPair` + `collideBoxesBatch(...)`                                                                                             |
| `sphere_sphere_batch_determinism_vs_single`     | DONE     | `test_sphere_sphere.cpp::sphere_sphere_batch_determinism_vs_single`                                                                                             |
| `capsule_capsule_batch_determinism_vs_single`   | DONE     | `test_capsule_capsule.cpp::capsule_capsule_batch_determinism_vs_single`                                                                                         |
| `cylinder_cylinder_batch_determinism_vs_single` | DONE     | `test_cylinder.cpp::cylinder_cylinder_batch_determinism_vs_single`                                                                                              |
| `convex_convex_batch_determinism_vs_single`     | DONE     | `test_convex.cpp::convex_convex_batch_determinism_vs_single`                                                                                                    |
| `mesh_mesh_batch_determinism_vs_single`         | DONE     | `test_mesh_mesh.cpp::mesh_mesh_batch_determinism_vs_single`                                                                                                     |
| `narrow_phase_collide_batch_dispatcher`         | DONE     | `test_narrow_phase.cpp::collide_batch_dispatcher_matches_scalar_loop` plus hit-flag and malformed-input coverage                                                |
| `bench_narrow_phase_per_pair_batch`             | DONE     | `.benchmark_results/native_collision_box_box_round7.json`: BoxBox/SphereSphere/CapsuleCapsule/CylinderCylinder/ConvexConvex/MeshMesh/Dispatcher N=1/10/100/1000 |
| `bench_simd_sat_axes_per_pair`                  | DEFERRED | Perf-wave benchmark: SoA SAT-axis projection across N pairs                                                                                                     |
| `bench_simd_face_clip_per_pair`                 | DEFERRED | Perf-wave benchmark: parallel polygon clipping                                                                                                                  |
| `bench_simd_xsimd_vs_scalar_speedup`            | DEFERRED | Cross-cutting: report per-pair speedup factor when SIMD-enabled                                                                                                 |
| `lint_no_naked_single_pair_loops`               | DEFERRED | Future lint: every new pair file must expose batch API too                                                                                                      |

## 5. Cross-Cutting Infrastructure

| Codename                                 | Status | Source                                                          |
| ---------------------------------------- | ------ | --------------------------------------------------------------- | --------------------------------------------------- |
| `fixture_shape_factories`                | DONE   | `tests/benchmark/collision/fixtures/shape_factories.hpp`        |
| `fixture_scene_builders`                 | DONE   | `tests/benchmark/collision/fixtures/scene_builders.hpp`         |
| `fixture_edge_cases`                     | DONE   | `tests/benchmark/collision/fixtures/edge_cases.hpp`             |
| `pixi_task_bm_collision_check`           | DONE   | `pixi.toml` `bm-collision-check` aggregator                     |
| `ctest_label_collision_native`           | DONE   | `tests/unit/collision/CMakeLists.txt`                           |
| `ctest_label_collision_native_stability` | DONE   | `tests/unit/CMakeLists.txt`, `tests/integration/CMakeLists.txt` | Labels the world-level native stability executables |
| `lint_runtime_isolation`                 | DONE   | `scripts/check_collision_runtime_isolation.py`                  |
| `lint_compat_facade_audit`               | DONE   | `scripts/audit_collision_compat_facades.py`                     |
| `lint_benchmark_schema`                  | DONE   | `scripts/check_collision_benchmarks.py`                         |

## Summary Counters (as of 2026-05-16)

- **§1 Pair-wise narrow-phase:** 89 DONE, 0 PARTIAL, 0 GAP (of 89 rows)
- **§2 Algorithm-level:** 38 DONE, 0 PARTIAL, 0 GAP, 11 DEFERRED (of 49 rows)
- **§3 Stress / regression:** 33 DONE, 0 PARTIAL, 0 GAP (of 33 rows)
- **§4 Benchmarks:** 30 DONE, 0 PARTIAL, 0 GAP, 14 DEFERRED (of 44 rows)
- **§5 Infrastructure:** 9 DONE, 0 PARTIAL, 0 GAP (of 9 rows)
- **TOTAL:** 199 DONE, 0 PARTIAL, 0 GAP, 25 DEFERRED (of 224 rows)

DART native has no open non-deferred feature-level rows in this matrix. The
remaining proposed-superset rows are explicitly deferred into:
(a) BVH/CCD scope-decision rows, (b) scale, scenario, and SIMD benchmark rows
for the follow-up performance wave, and (c) future scalar/infrastructure
variants.

## Next Priorities (TDD order)

1. **Algorithm-isolation deferred rows** — decide whether any remaining
   BVH/CCD variants belong in DART native before the performance wave.
2. **Scale/scenario benchmark rows** — broadphase scaling, stacked boxes,
   ragdoll piles, convex-vs-landscape, and BVH build/traversal benchmarks.
   These provide baselines for the next performance wave.
3. **Stretch deferred rows** — auto-diff narrow-phase, float/double parity,
   k-DOP / OBB / RSS BVH variants. These are scope-decision items, not
   urgent fills. Decide before adding rows.

Update this matrix whenever a row flips DONE / PARTIAL / GAP. Keep the
summary counters at the bottom in sync.
