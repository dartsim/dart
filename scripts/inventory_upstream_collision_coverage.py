#!/usr/bin/env python3
"""Generate an upstream collision coverage inventory.

The inventory is a working artifact for long-running native-collision coverage
work. It records upstream FCL, Bullet, and ODE tests/benchmarks from local
clones under .deps so DART coverage can be mapped without relying on memory.
"""

from __future__ import annotations

import argparse
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

DEFAULT_SOURCE_ROOT = Path(".deps/upstream-collision-sources")
DEFAULT_OUTPUT = Path(
    "docs/dev_tasks/native_collision_upstream_superset/01-upstream-inventory.md"
)
DEFAULT_CASE_MAP_OUTPUT = Path(
    "docs/dev_tasks/native_collision_upstream_superset/03-case-map.md"
)
SOURCE_EXTENSIONS = {".c", ".cc", ".cpp", ".h", ".hpp"}
CASE_PATTERN = re.compile(
    r"\b(GTEST_TEST|TEST|TEST_F|TEST_P|TYPED_TEST|TEST_FIXTURE)"
    r"\s*\(\s*([^,\)\s]+)(?:\s*,\s*([^\)\s]+))?\s*\)"
)


@dataclass(frozen=True)
class Scope:
    project: str
    name: str
    path: str
    role: str
    native_scope: str
    include_cases: bool = True
    source_only: bool = False
    exclude_parts: tuple[str, ...] = ()
    only_suffixes: tuple[str, ...] = ()
    exclude_names: tuple[str, ...] = ()


@dataclass(frozen=True)
class Case:
    source: Path
    kind: str
    suite: str
    name: str


@dataclass
class ScopeInventory:
    scope: Scope
    files: list[Path]
    cases: list[Case]
    benchmark_cases: list[str]


SCOPES = (
    Scope(
        "FCL",
        "FCL test tree",
        "fcl/test",
        "Correctness tests and geometry/narrowphase/broadphase coverage",
        "native-collision correctness superset",
        exclude_parts=("libsvm",),
    ),
    Scope(
        "Bullet",
        "Bullet collision unit test",
        "bullet3/test/collision",
        "GTest collision correctness",
        "native-collision correctness superset",
    ),
    Scope(
        "Bullet",
        "Bullet collision examples",
        "bullet3/examples/Collision",
        "Collision SDK/tutorial source scenarios",
        "native-collision scenario coverage",
        include_cases=False,
        source_only=True,
    ),
    Scope(
        "Bullet",
        "Bullet raycast example",
        "bullet3/examples/Raycast",
        "Raycast scenario source",
        "native-collision raycast scenario coverage",
        include_cases=False,
        source_only=True,
    ),
    Scope(
        "Bullet",
        "Bullet benchmark examples",
        "bullet3/examples/Benchmarks",
        "Benchmark scenario source",
        "native-collision benchmark superset",
        include_cases=False,
        source_only=True,
    ),
    Scope(
        "Bullet",
        "Bullet OpenCL collision kernels",
        "bullet3/test/OpenCL/AllBullet3Kernels",
        "Broadphase, narrowphase, raycast, and contact kernel tests",
        "native-collision GPU/prototype follow-up",
        only_suffixes=(".cpp",),
    ),
    Scope(
        "Bullet",
        "Bullet other tests",
        "bullet3/test",
        "Full upstream test tree accounted outside native collision detector",
        "catalogued outside native-collision scope",
        exclude_parts=(
            "gtest-1.7.0",
            "collision",
            "OpenCL/AllBullet3Kernels",
        ),
    ),
    Scope(
        "ODE",
        "ODE collision unit tests",
        "ode/tests",
        "Core collision and point-depth tests",
        "native-collision correctness superset",
        only_suffixes=(".cpp",),
        exclude_names=("friction.cpp", "joint.cpp", "odemath.cpp", "main.cpp"),
        exclude_parts=("joints", "UnitTest++"),
    ),
    Scope(
        "ODE",
        "ODE other core tests",
        "ode/tests",
        "Joint, friction, and math tests accounted outside native collision detector",
        "catalogued outside native-collision scope",
        only_suffixes=(".cpp",),
        exclude_names=("collision.cpp", "collision_point_depth.cpp", "main.cpp"),
        exclude_parts=("UnitTest++",),
    ),
    Scope(
        "ODE",
        "ODE demos",
        "ode/ode/demo",
        "Interactive demo scenarios, including collision and trimesh demos",
        "native-collision scenario/benchmark candidates",
        include_cases=False,
        source_only=True,
    ),
    Scope(
        "ODE",
        "ODE libccd tests",
        "ode/libccd/src/testsuites",
        "libccd GJK/MPR/EPA correctness tests bundled with ODE",
        "native-collision correctness superset",
        only_suffixes=(".c",),
        exclude_names=("main.c", "common.c", "support.c", "bench.c", "bench2.c"),
    ),
    Scope(
        "ODE",
        "ODE libccd benchmarks",
        "ode/libccd/src/testsuites",
        "libccd benchmark programs bundled with ODE",
        "native-collision benchmark superset",
        include_cases=False,
        source_only=True,
        only_suffixes=(".c",),
        exclude_names=(
            "main.c",
            "common.c",
            "support.c",
            "boxbox.c",
            "boxcyl.c",
            "cylcyl.c",
            "mpr_boxbox.c",
            "mpr_boxcyl.c",
            "mpr_cylcyl.c",
            "polytope.c",
            "spheresphere.c",
            "vec3.c",
        ),
    ),
)

BULLET_BENCHMARK_CASES = (
    "Benchmark 1: large box grid/stack stress scenario",
    "Benchmark 2: pyramid, wall, and tower compound box scenario",
    "Benchmark 3: scaled stack scenario",
    "Benchmark 4: convex hull Taru stack scenario",
    "Benchmark 5: mixed box/sphere/capsule stacks",
    "Benchmark 6: convex hull fall scenario",
    "Benchmark 7: convex hull fall plus 500 moving raycasts",
    "Benchmark 8: Halton Voronoi convex-cell scenario",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--case-map-output", type=Path, default=DEFAULT_CASE_MAP_OUTPUT)
    return parser.parse_args()


def git_value(repo: Path, *args: str) -> str:
    if not (repo / ".git").exists():
        return "missing"
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.strip()


def rel_parts(path: Path, root: Path) -> tuple[str, ...]:
    return path.relative_to(root).parts


def excluded(path: Path, root: Path, scope: Scope) -> bool:
    parts = "/".join(rel_parts(path, root))
    if any(part in rel_parts(path, root) for part in scope.exclude_parts):
        return True
    if any(excluded_part in parts for excluded_part in scope.exclude_parts):
        return True
    return path.name in scope.exclude_names


def collect_files(source_root: Path, scope: Scope) -> list[Path]:
    root = source_root / scope.path
    if not root.exists():
        return []
    files = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or excluded(path, root, scope):
            continue
        if path.name in {"CMakeLists.txt", "Makefile.am"}:
            files.append(path)
            continue
        if path.suffix not in SOURCE_EXTENSIONS:
            continue
        if scope.only_suffixes and path.suffix not in scope.only_suffixes:
            continue
        files.append(path)
    return files


def collect_cases(source_root: Path, scope: Scope, files: list[Path]) -> list[Case]:
    if not scope.include_cases:
        return []
    cases = []
    for path in files:
        if path.suffix not in SOURCE_EXTENSIONS:
            continue
        text = path.read_text(errors="ignore")
        for match in CASE_PATTERN.finditer(text):
            kind = match.group(1)
            suite = match.group(2)
            name = match.group(3) or ""
            if suite in {"name"}:
                continue
            cases.append(Case(path.relative_to(source_root), kind, suite, name))
    return cases


def benchmark_cases(scope: Scope) -> list[str]:
    if scope.name == "Bullet benchmark examples":
        return list(BULLET_BENCHMARK_CASES)
    if scope.name == "ODE libccd benchmarks":
        return ["bench.c", "bench2.c"]
    return []


def inventories(source_root: Path) -> list[ScopeInventory]:
    result = []
    for scope in SCOPES:
        files = collect_files(source_root, scope)
        cases = collect_cases(source_root, scope, files)
        result.append(ScopeInventory(scope, files, cases, benchmark_cases(scope)))
    return result


def source_table(source_root: Path) -> list[str]:
    repos = (
        ("FCL", source_root / "fcl"),
        ("Bullet", source_root / "bullet3"),
        ("ODE", source_root / "ode"),
    )
    lines = [
        "| Project | Origin | Revision |",
        "| --- | --- | --- |",
    ]
    for project, repo in repos:
        origin = git_value(repo, "remote", "get-url", "origin")
        revision = git_value(repo, "rev-parse", "HEAD")
        lines.append(f"| {project} | `{origin}` | `{revision}` |")
    return lines


def format_path(path: Path, source_root: Path) -> str:
    return path.relative_to(source_root).as_posix()


def case_display(case: Case) -> str:
    if case.name:
        return f"{case.kind}({case.suite}, {case.name})"
    return f"{case.kind}({case.suite})"


CASE_MAP_OVERRIDES = {
    (
        "FCL",
        "fcl/test/narrowphase/detail/test_collision_func_matrix.cpp",
        "GTEST_TEST(CollisionFuncMatrix, LibCccdSolverSupport)",
    ): (
        "covered",
        "tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.IsSupported; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.SupportedShapePairs; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.EllipsoidShapeAdapter",
        "DART has one native dispatcher rather than FCL's solver-specific collision matrix. Native pair support is checked by the public isSupported matrix, supported-shape collision tests, and adapter tests for shapes represented as native convex/sphere geometry.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/test_collision_func_matrix.cpp",
        "GTEST_TEST(CollisionFuncMatrix, IndepSolverSupport)",
    ): (
        "covered",
        "tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.IsSupported; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.SupportedShapePairs; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.EllipsoidShapeAdapter",
        "DART uses a single native narrowphase dispatcher instead of FCL's independent/libccd solver split; the public native support matrix and adapted shape coverage exercise the equivalent supported-pair surface.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/test_failed_at_this_configuration.cpp",
        "GTEST_TEST(ConfigurationFailureTest, ConfirmFormatting)",
    ): (
        "not-applicable",
        "FCL diagnostic formatting API",
        "This row checks FCL's internal exception message formatter for failed configurations, not collision detector behavior exposed by DART native collision.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_auto_diff.cpp",
        "GTEST_TEST(FCL_AUTO_DIFF, basic)",
    ): (
        "not-applicable",
        "native collision scalar contract",
        "FCL validates AutoDiffScalar support in its templated distance solver. DART native collision exposes double-precision public collision, distance, and raycast queries, not an autodiff collision API.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_bvh_models.cpp",
        "GTEST_TEST(FCL_BVH_MODELS, building_bvh_models)",
    ): (
        "not-applicable",
        "FCL BVHModel build API",
        "This row checks FCL BVHModel construction states for multiple bounding-volume families. DART native collision does not expose that BVHModel API; public mesh construction and mesh collision are covered separately.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_collision.cpp",
        "GTEST_TEST(FCL_COLLISION, test_SplineMotion_rotated_spline_collide_test)",
    ): (
        "not-applicable",
        "continuous spline collision API",
        "This row exercises FCL continuous collision between spline-motion objects. DART native collision currently exposes discrete collision, distance, and raycast queries, not FCL's continuous spline-motion API.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_collision.cpp",
        "GTEST_TEST(FCL_COLLISION, OBB_Box_test)",
    ): (
        "not-applicable",
        "FCL OBB bounding-volume predicate",
        "This row compares FCL OBB overlap predicates with exact box collision. DART native broadphase uses AABBs and does not expose FCL OBB predicates; exact rotated box collision is covered by native box-box tests.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_collision.cpp",
        "GTEST_TEST(FCL_COLLISION, OBB_shape_test)",
    ): (
        "not-applicable",
        "FCL OBB bounding-volume predicate",
        "This row checks conservative FCL OBB overlap predicates for primitive envelopes. DART native collision does not expose OBB bounding-volume tests; exact primitive collision and adapter rows cover the public behavior.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_collision.cpp",
        "GTEST_TEST(FCL_COLLISION, OBB_AABB_test)",
    ): (
        "not-applicable",
        "FCL OBB bounding-volume predicate",
        "This row compares FCL OBB and AABB overlap predicates. DART native exposes AABB broadphase behavior, but not FCL OBB predicates, so the OBB comparison is outside native collision scope.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_collision.cpp",
        "GTEST_TEST(FCL_COLLISION, mesh_mesh)",
    ): (
        "covered",
        "tests/unit/collision/test_mesh_mesh.cpp::MeshMesh.SingleTriangleAndLargeMesh; tests/unit/collision/test_mesh_mesh.cpp::MeshMeshBatch.mesh_mesh_batch_determinism_vs_single; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.MeshMeshContactsPreserveTriangleIds",
        "Checks native mesh-mesh collision on single-triangle and larger meshes, preserves triangle identifiers, and verifies batch traversal matches scalar mesh collision. FCL split-method and BV-family variants are implementation details.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_constant_eps.cpp",
        "GTEST_TEST(FCL_CONSTANTS_EPS, precision_dependent)",
    ): (
        "not-applicable",
        "FCL math constants API",
        "This row validates FCL's scalar-specific epsilon constants, not native collision detector geometry, query, or dispatch behavior.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_constant_eps.cpp",
        "GTEST_TEST(FCL_CONSTANTS_EPS, autodiff_compatibility)",
    ): (
        "not-applicable",
        "FCL math constants API",
        "This row validates FCL constants return types for AutoDiffScalar, while DART native collision exposes double-precision collision APIs and no autodiff constants contract.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_frontlist.cpp",
        "GTEST_TEST(FCL_FRONT_LIST, front_list)",
    ): (
        "not-applicable",
        "FCL front-list traversal cache",
        "This row checks FCL's BVH front-list acceleration path against ordinary traversal. DART native mesh collision does not expose front-list traversal state; public mesh collision behavior is covered by mesh-mesh tests.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_general.cpp",
        "GTEST_TEST(FCL_GENERAL, general)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.Rotated45_Diagonal; tests/unit/collision/test_box_box.cpp::BoxBox.RotatedFacePatchReductionRespectsContactLimit; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.BoxBoxEdgeEdgeContact",
        "The upstream row only prints contacts from a rotated box-box collision. DART native rotated box contact behavior is covered with asserted collision, contact depth, normal, and edge/face cases.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_generate_bvh_model_deferred_finalize.cpp",
        "GTEST_TEST(FCL_GENERATE_BVH_MODELS, generating_bvh_models_from_primitives)",
    ): (
        "not-applicable",
        "FCL geometric_shape_to_BVH_model API",
        "This row checks FCL's deferred BVHModel finalization API while converting primitives into mesh data. DART native collision does not expose deferred BVH model construction; primitive adapters and mesh construction are covered separately.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, sphere_shape)",
    ): (
        "not-applicable",
        "native collision shape API",
        "This row checks FCL Sphere::computeVolume. DART native collision shapes expose query geometry and bounds for collision, not a native collision volume API.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, gjkcache)",
    ): (
        "not-applicable",
        "FCL cached GJK guess API",
        "This row compares FCL collision results with and without a cached GJK guess. DART native collision does not expose cached GJK guesses as a public correctness contract.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_simple.cpp",
        "GTEST_TEST(FCL_SIMPLE, Vec_nf_test)",
    ): (
        "not-applicable",
        "FCL math and sampler utilities",
        "This row exercises FCL VectorN and sampler helper utilities, not DART native collision detector behavior.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_simple.cpp",
        "GTEST_TEST(FCL_SIMPLE, projection_test_line)",
    ): (
        "not-applicable",
        "FCL private projection helper",
        "This row checks FCL's standalone line projection helper. DART native collision exposes public distance and witness queries instead of this helper API.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_simple.cpp",
        "GTEST_TEST(FCL_SIMPLE, projection_test_triangle)",
    ): (
        "not-applicable",
        "FCL private projection helper",
        "This row checks FCL's standalone triangle projection helper. DART native collision exposes public distance and witness queries instead of this helper API.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_simple.cpp",
        "GTEST_TEST(FCL_SIMPLE, projection_test_tetrahedron)",
    ): (
        "not-applicable",
        "FCL private projection helper",
        "This row checks FCL's standalone tetrahedron projection helper. DART native collision exposes public distance and witness queries instead of this helper API.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_cylinder_half_space.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_cylinder_half_space_libccd)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderPlane.HalfspaceDepthAcrossAxisDirections; tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms",
        "Checks cylinder penetration depth against DART's one-sided contact plane/halfspace with the cylinder axis in both directions. DART has one native dispatcher rather than FCL's solver split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_cylinder_half_space.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, collision_cylinder_half_space_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderPlane.HalfspaceDepthAcrossAxisDirections; tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms",
        "Checks cylinder penetration depth against DART's one-sided contact plane/halfspace with the cylinder axis in both directions. DART uses a single native dispatcher instead of FCL's independent/libccd solver split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_distance.cpp",
        "GTEST_TEST(FCL_DISTANCE, mesh_distance)",
    ): (
        "covered",
        "tests/unit/collision/test_mesh_mesh.cpp::MeshMesh.SingleTriangleAndLargeMesh; tests/unit/collision/test_mesh_mesh.cpp::MeshMesh.SeparatedDistanceWitnessesAcrossPairOrder",
        "Checks native mesh-mesh distance for overlapping and separated meshes with finite witnesses and swapped pair order. FCL's BV-family and split-method knobs are implementation details not exposed by DART native collision.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_distance.cpp",
        "GTEST_TEST(FCL_DISTANCE, NearestPointFromDegenerateSimplex)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceBoxBox.DegenerateSimplexWitnessRegression; tests/unit/collision/test_gjk.cpp::GjkSignedDistance.RotatedBoxEdgeFaceCases",
        "Checks the historical degenerate-simplex rotated box-box distance configuration with finite witness points and the expected positive separation.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spheretriangle)",
    ): (
        "covered",
        "tests/unit/collision/test_mesh_mesh.cpp::PrimitiveMesh.SphereVsMesh; tests/unit/collision/test_mesh_mesh.cpp::PrimitiveMesh.SphereTouchesSharedTriangleEdgeAtZeroDepth",
        "DART exposes triangle collision through MeshShape; checks sphere/triangle-mesh penetration, transformed mesh frames, and exact zero-depth edge contact.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spheretriangle)",
    ): (
        "covered",
        "tests/unit/collision/test_mesh_mesh.cpp::PrimitiveMesh.SphereVsMesh; tests/unit/collision/test_mesh_mesh.cpp::PrimitiveMesh.SphereTouchesSharedTriangleEdgeAtZeroDepth",
        "DART exposes triangle collision through MeshShape rather than FCL's direct shape-triangle GJK helper; public sphere/triangle-mesh contact behavior is covered with penetration and exact edge-touching cases.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_halfspacetriangle)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_plane.cpp::PlaneMesh.Penetrating",
        "Checks triangle collision against DART's one-sided contact plane/halfspace across transformed common frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_planetriangle)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane is a two-sided infinite plane in this direct shape-triangle GJK row. DART native PlaneShape is a one-sided contact plane/halfspace, so the exact two-sided plane semantic is outside the native public primitive set.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_box.cpp",
        "GTEST_TEST(FCL_SPHERE_BOX, LargBoxSmallSphere_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsWithIncompatibleScaleRatios; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks long-skinny-box/small-sphere face contact depth/normal and verifies the native dispatcher preserves the same scale-sensitive contact when the pair order is swapped. DART has one native dispatcher rather than FCL's solver split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_box.cpp",
        "GTEST_TEST(FCL_SPHERE_BOX, LargBoxSmallSphere_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsWithIncompatibleScaleRatios; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks long-skinny-box/small-sphere face contact depth/normal and verifies the native dispatcher preserves the same scale-sensitive contact when the pair order is swapped. DART uses a single native dispatcher instead of FCL's independent/libccd solver split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames",
        "Checks large capsule-sphere endcap separation along the capsule axis across identity and transformed capsule frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_z_negative)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames",
        "Checks large capsule-sphere endcap separation on the opposite capsule axis side across identity and transformed capsule frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_x)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames",
        "Checks large capsule-sphere side separation in identity and transformed common frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_separated_capsule_rotated)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks large capsule-sphere separation under transformed capsule frames and verifies rotated capsule-sphere dispatcher contact behavior with swapped pair order.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks large capsule-sphere endcap penetration depth/finite contacts and dispatcher pair-order preservation for the same scale-sensitive primitive family.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Intersect_test_penetration_z_rotated)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks large capsule-sphere endcap penetration across transformed capsule frames and a rotated capsule dispatcher case with asserted depth and swapped-order normal handling.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_collision)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleSphere.LargeSphereSignedDistanceAlongCapsuleAxis",
        "DART reports capsule-sphere collision through negative signed distance; this test checks large-radius axis-aligned endcap penetration under identity and rotated capsule frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_capsule.cpp",
        "GTEST_TEST(FCL_SPHERE_CAPSULE, Sphere_Capsule_Distance_test_separated)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleSphere.LargeSphereSignedDistanceAlongCapsuleAxis",
        "Checks the corresponding large-radius capsule-sphere positive signed distance along both capsule-axis endcaps under identity and rotated frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_cylinder.cpp",
        "GTEST_TEST(FCL_SPHERE_CYLINDER, LargCylinderSmallSphere_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderSphere.ContactsWithIncompatibleScaleRatios; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks large-disk/tiny-sphere face and barrel contacts/separations across frames and verifies dispatcher pair-order preservation for the scale-sensitive face-contact case. DART has one native dispatcher rather than FCL's solver split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_cylinder.cpp",
        "GTEST_TEST(FCL_SPHERE_CYLINDER, LargBoxSmallSphere_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderSphere.ContactsWithIncompatibleScaleRatios; tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.PrimitiveDispatcherPreservesScaleContactsUnderPairOrder",
        "Checks large-disk/tiny-sphere face and barrel contacts/separations across frames and verifies dispatcher pair-order preservation for the scale-sensitive face-contact case. DART uses a single native dispatcher instead of FCL's independent/libccd solver split.",
    ),
    (
        "Bullet",
        "bullet3/test/collision/main.cpp",
        "TEST(BulletCollisionTest, GjkMPRSphereSphereDistance)",
    ): (
        "covered",
        "tests/unit/collision/test_libccd_algorithms.cpp::GjkLibccd.SphereSphereSeparationDistance; tests/unit/collision/test_libccd_algorithms.cpp::GjkLibccd.SphereSphereMprMatchesLibccd; tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks sphere-sphere separation and penetration distance behavior through native GJK/MPR and public signed-distance witnesses.",
    ),
    (
        "Bullet",
        "bullet3/test/collision/main.cpp",
        "TEST(BulletCollisionTest, GjkEpaSphereSphereDistance)",
    ): (
        "covered",
        "tests/unit/collision/test_libccd_algorithms.cpp::GjkLibccd.SphereSphereSeparationDistance; tests/unit/collision/test_libccd_algorithms.cpp::GjkLibccd.SphereSphereEpaOrMprMatchesLibccd; tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks sphere-sphere separation and penetration distance behavior through native GJK plus EPA/MPR fallback and public signed-distance witnesses.",
    ),
    (
        "Bullet",
        "bullet3/test/collision/main.cpp",
        "TEST(BulletCollisionTest, GjkEpaSphereSphereRadiusNotFullMarginDistance)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.ZeroRadiusCoincident; tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "DART native spheres store radius directly instead of Bullet's radius/margin split; the same zero-radius, separated, and penetrating sphere distance behavior is covered by public native distance tests.",
    ),
    (
        "Bullet",
        "bullet3/test/collision/main.cpp",
        "TEST(BulletCollisionTest, AnalyticSphereSphereDistance)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.ZeroRadiusCoincident; tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.ScaleRegimes; tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks analytic sphere-sphere distance over zero radius, small/large radius scales, separated cases, penetrating cases, swapped order, and transformed frames.",
    ),
    (
        "Bullet",
        "bullet3/test/collision/main.cpp",
        "TEST(BulletCollisionTest, Heightfield_ProcessAllTriangles_FiltersByUpAxis)",
    ): (
        "covered",
        "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.HeightmapShapeAdapter; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.RaycastWorksForHeightmap; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.RaycastHeightmapMissesOutsideVerticalExtent",
        "DART adapts heightmaps into native mesh geometry instead of exposing Bullet's triangle-callback API; adapter, vertical hit, and below-heightfield miss behavior cover the public native heightmap collision path.",
    ),
    (
        "ODE",
        "ode/tests/collision.cpp",
        "TEST(test_collision_trimesh_sphere_exact)",
    ): (
        "covered",
        "tests/unit/collision/test_mesh_mesh.cpp::PrimitiveMesh.SphereTouchesSharedTriangleEdgeAtZeroDepth",
        "Checks a sphere exactly touching the shared diagonal edge of a two-triangle square mesh with zero penetration depth, including translated and rotated mesh frames.",
    ),
    (
        "ODE",
        "ode/tests/collision.cpp",
        "TEST(test_collision_heightfield_ray_fail)",
    ): (
        "covered",
        "tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.RaycastHeightmapMissesOutsideVerticalExtent; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.RaycastHeightmapMissesOutsideGridBounds",
        "Checks heightmap ray misses outside the active vertical extent and outside the grid bounds through DART's public heightmap raycast path without producing stale hits.",
    ),
    (
        "ODE",
        "ode/tests/collision.cpp",
        "TEST(test_collision_ray_convex)",
    ): (
        "covered",
        "tests/unit/collision/test_raycast_core.cpp::RaycastConvex.RespectsRayAndShapeTransforms",
        "Checks convex raycasts with centered and offset rays, rotated convex frames, misses caused by rotation, and translated shape/ray coordinates.",
    ),
    (
        "ODE",
        "ode/tests/collision_point_depth.cpp",
        "TEST(test_collision_sphere_point_depth)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.PointDepthSamplesAcrossInsideSurfaceOutside",
        "Checks point depth as the negated signed distance between a sphere and a zero-radius point across center, inside, surface, and outside samples.",
    ),
    (
        "ODE",
        "ode/tests/collision_point_depth.cpp",
        "TEST(test_collision_box_point_depth)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.PointDepthSamplesAcrossInsideSurfaceOutside",
        "Checks point depth as the negated signed distance between a box and a zero-radius point across face, edge, corner, inside, surface, and outside samples.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacetriangle)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_plane.cpp::PlaneMesh.Penetrating",
        "Checks a triangle crossing a one-sided contact plane, including translated/rotated common frames and both pair orders.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacesphere)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_plane.cpp::PlaneSphere.Penetrating; tests/unit/collision/test_plane.cpp::PlaneSphere.NoCollision",
        "Checks sphere support depth and separation against DART's one-sided contact plane across identity and transformed common frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacebox)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_plane.cpp::PlaneBox.Penetrating; tests/unit/collision/test_plane.cpp::PlaneBox.NoCollision; tests/unit/collision/test_plane.cpp::PlaneBox.RotatedBox",
        "Checks box support depth, separation, rotated orientation, transformed common frames, and both pair orders against DART's one-sided contact plane.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspaceellipsoid)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.EllipsoidShapeAdapter; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.SphericalEllipsoidShapeAdapter",
        "DART adapts ellipsoids into native sphere/convex support geometry; checks anisotropic convex support depth/separation against the contact plane plus adapter bounds.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacecapsule)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_plane.cpp::PlaneCapsule.Standing; tests/unit/collision/test_plane.cpp::PlaneCapsule.Lying; tests/unit/collision/test_plane.cpp::PlaneCapsule.NoCollision",
        "Checks capsule support depth and separation against DART's one-sided contact plane for standing, lying, transformed common-frame, and pair-order cases.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacecylinder)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_cylinder.cpp::CylinderPlane.CylinderStandingOnPlane; tests/unit/collision/test_cylinder.cpp::CylinderPlane.CylinderLyingOnPlane; tests/unit/collision/test_cylinder.cpp::CylinderPlane.CylinderTiltedOnPlane; tests/unit/collision/test_cylinder.cpp::CylinderPlane.NoCollision",
        "Checks cylinder support depth and separation against DART's one-sided contact plane for axial, lying, tilted, transformed common-frame, and pair-order cases.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_halfspacecone)",
    ): (
        "covered",
        "tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter",
        "DART adapts cones into native convex support geometry; checks convex support depth/separation against the contact plane plus cone adapter bounds.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planetriangle)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane is a two-sided infinite plane. DART native PlaneShape is a one-sided contact plane/halfspace primitive, so FCL's two-sided plane collision row has no equivalent native public primitive; the corresponding one-sided triangle case is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planesphere)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane checks symmetric two-sided sphere separation. DART native PlaneShape is one-sided, so this exact plane semantic is outside the native public primitive set; one-sided sphere contact/separation is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planebox)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane checks symmetric two-sided box separation. DART native PlaneShape is one-sided, so this exact plane semantic is outside the native public primitive set; one-sided box contact/separation is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planeellipsoid)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane checks symmetric two-sided ellipsoid separation. DART native PlaneShape is one-sided, so this exact plane semantic is outside the native public primitive set; one-sided adapted-convex contact/separation is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planecapsule)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane checks symmetric two-sided capsule separation. DART native PlaneShape is one-sided, so this exact plane semantic is outside the native public primitive set; one-sided capsule contact/separation is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planecylinder)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane checks symmetric two-sided cylinder separation. DART native PlaneShape is one-sided, so this exact plane semantic is outside the native public primitive set; one-sided cylinder contact/separation is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_planecone)",
    ): (
        "not-applicable",
        "DART native PlaneShape halfspace semantics",
        "FCL Plane checks symmetric two-sided cone separation. DART native PlaneShape is one-sided, so this exact plane semantic is outside the native public primitive set; one-sided adapted-convex contact/separation is covered by the halfspace row.",
    ),
    (
        "FCL",
        "fcl/test/geometry/shape/test_capsule.cpp",
        "GTEST_TEST(Capsule, LocalAABBComputation_Capsule)",
    ): (
        "covered",
        "tests/unit/collision/test_shapes.cpp::CapsuleShape.ComputeLocalAabbAcrossRadiusHeightCases",
        "Checks capsule local-AABB half extents across radius/height aspect ratios.",
    ),
    (
        "FCL",
        "fcl/test/geometry/shape/test_convex.cpp",
        "GTEST_TEST(ConvexGeometry, SupportVertexCoPlanarFaces)",
    ): (
        "covered",
        "tests/unit/collision/test_shapes.cpp::ConvexShape.SupportMatchesExhaustiveVertexSearch",
        "Checks co-planar/tied support directions against exhaustive vertex search.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/convexity_based_algorithm/test_gjk_libccd-inl_signed_distance.cpp",
        "GTEST_TEST(FCL_GJKSignedDistance, sphere_sphere)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks separated and penetrating sphere signed distances, boundary witnesses, swapped order, and transformed-frame invariance.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/convexity_based_algorithm/test_gjk_libccd-inl_signed_distance.cpp",
        "GTEST_TEST(FCL_GJKSignedDistance, box_box)",
    ): (
        "covered",
        "tests/unit/collision/test_gjk.cpp::GjkSignedDistance.RotatedBoxEdgeFaceCases",
        "Checks rotated box edge-face separation, touching, and penetration through GJK plus penetration recovery across translated/rotated frames.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spheresphere)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_sphere.cpp::SphereSphere.LargeRadiiBoundaryAndPenetrationAcrossFrames",
        "Checks large-radius sphere separation, touching, shallow penetration, coincident centers, negative-axis symmetry, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_boxbox)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.Coincident; tests/unit/collision/test_box_box.cpp::BoxBox.Overlapping_AlongX; tests/unit/collision/test_box_box.cpp::BoxBox.Rotated45_Diagonal; tests/unit/collision/test_box_box.cpp::BoxBox.RotatedBoxOnFlatGroundEmitsFacePatch",
        "Checks coincident, separated boundary, translated overlap, rotated overlap, and contact-patch behavior for box-box intersections.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spherebox)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.SeparatedAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsAcrossBoxFramesAndSphereOrientations",
        "Checks sphere-box coincident, touching, shallow penetration, separation, rotated/translated common frames, and sphere-orientation invariance.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_spherecapsule)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames",
        "Checks large-radius capsule-sphere coincident, touching, shallow penetration, separation, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersection_cylindercylinder)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderCylinder.EqualRadiusBoundaryAndSeparationAcrossFrames",
        "Checks equal-radius cylinder coincident, side-touching, shallow side penetration, side separation, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_spheresphere)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks large-radius sphere signed-distance gaps, penetration, swapped order, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_boxbox)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceBoxBox.SignedDistanceWithCommonFrames",
        "Checks same-orientation box signed distances for coincident penetration, positive gaps, diagonal gaps, swapped order, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistance_boxsphere)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.LargeSphereSignedDistanceWithCommonFrames",
        "Checks large-sphere/box signed distance for coincident penetration, small and large positive gaps, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spheresphere)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_sphere.cpp::SphereSphere.LargeRadiiBoundaryAndPenetrationAcrossFrames; tests/unit/collision/test_gjk.cpp::Gjk.SpheresSeparated; tests/unit/collision/test_gjk.cpp::Gjk.SpheresIntersecting",
        "Checks the sphere-sphere boundary cases through DART's public native collision path and direct GJK separated/intersecting queries.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_boxbox)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.Rotated45_Diagonal; tests/unit/collision/test_gjk.cpp::Gjk.BoxesSeparated; tests/unit/collision/test_gjk.cpp::Gjk.BoxesIntersecting; tests/unit/collision/test_gjk.cpp::GjkSignedDistance.RotatedBoxEdgeFaceCases",
        "Checks box-box separated/intersecting GJK queries plus rotated edge-face boundary and penetration configurations.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spherebox)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_gjk.cpp::Gjk.SphereBoxSeparated; tests/unit/collision/test_gjk.cpp::Gjk.SphereBoxIntersecting; tests/unit/collision/test_gjk.cpp::Gjk.SphereBoxTouching",
        "Checks sphere-box common-frame contacts and direct GJK separated, touching, and penetrating queries.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_spherecapsule)",
    ): (
        "covered",
        "tests/unit/collision/test_capsule_capsule.cpp::CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames",
        "Checks capsule-sphere coincident, touching, shallow penetration, separation, and common-frame transforms through DART's native capsule-sphere path.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeIntersectionGJK_cylindercylinder)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderCylinder.EqualRadiusBoundaryAndSeparationAcrossFrames",
        "Checks equal-radius cylinder coincident, touching, shallow penetration, separation, and common-frame transforms through DART's native cylinder-cylinder path.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_spheresphere)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks large-radius sphere signed-distance gaps, penetration, swapped order, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_boxbox)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceBoxBox.SignedDistanceWithCommonFrames",
        "Checks same-orientation box signed distances for coincident penetration, positive gaps, swapped order, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_geometric_shapes.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, shapeDistanceGJK_boxsphere)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.LargeSphereSignedDistanceWithCommonFrames",
        "Checks large-sphere/box signed distance for coincident penetration, small and large positive gaps, and common-frame transforms.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_box_box.cpp",
        "GTEST_TEST(FCL_BOX_BOX, collision_box_box_all_contacts_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.CubeFacePatchSingleContactAcrossEquivalentOrientations",
        "Checks a rotated cube against an equivalent-orientation cube face with a single requested contact, preserving depth/normal, argument-order, and cube-symmetry invariants.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_box_box.cpp",
        "GTEST_TEST(FCL_BOX_BOX, collision_box_box_all_contacts_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.CubeFacePatchSingleContactAcrossEquivalentOrientations",
        "Checks the same rotated-cube face-patch scenario through DART's native SAT/contact path; DART has one native box-box solver rather than upstream's solver-type split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_box_box.cpp",
        "GTEST_TEST(FCL_BOX_BOX, collision_box_box_cull_contacts_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.RotatedFacePatchReductionRespectsContactLimit",
        "Checks a rotated face-patch overlap whose contact polygon must be reduced, including depth/normal, contact-limit, argument-order, and cube-symmetry invariants.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_box_box.cpp",
        "GTEST_TEST(FCL_BOX_BOX, collision_box_box_cull_contacts_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.RotatedFacePatchReductionRespectsContactLimit",
        "Checks the same contact-reduction face-patch scenario through DART's native SAT/contact path; DART has one native box-box solver rather than upstream's solver-type split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_box_box.cpp",
        "GTEST_TEST(FCL_BOX_BOX, collision_box_box_edge_contact_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.EdgeEdgeContactAcrossEquivalentCubeOrientations",
        "Checks a rotated cube edge-edge overlap with a single support contact, preserving depth, position, normal, argument-order, and cube-symmetry invariants.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_box_box.cpp",
        "GTEST_TEST(FCL_BOX_BOX, collision_box_box_edge_contact_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_box_box.cpp::BoxBox.EdgeEdgeContactAcrossEquivalentCubeOrientations",
        "Checks the same edge-edge cube scenario through DART's native SAT/contact path; DART has one native box-box solver rather than upstream's solver-type split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_sphere.cpp",
        "GTEST_TEST(FCL_SPHERE_SPHERE, Separating_Spheres_INDEP)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets; tests/unit/collision/test_gjk.cpp::Gjk.SpheresSeparated",
        "Checks the same four-sphere separated distance matrix with nearest-point witnesses, swapped order, and positive signed distances through DART's native distance path plus direct GJK separation.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_sphere_sphere.cpp",
        "GTEST_TEST(FCL_SPHERE_SPHERE, Separating_Spheres_LIBCCD)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets; tests/unit/collision/test_libccd_algorithms.cpp::GjkLibccd.SphereSphereSeparationDistance",
        "Checks the same four-sphere separated distance matrix through DART's native distance witnesses and libccd-style sphere-sphere GJK separation anchor.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks large sphere-sphere positive gaps and penetrations, including the exact axial and diagonal signed-distance configurations with nearest-point witnesses.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_sphere_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets",
        "Checks the same large sphere-sphere signed-distance configurations through DART's single native signed-distance path.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleSphere.LargeSphereSignedDistanceAlongCapsuleSide",
        "Checks large capsule-sphere side separation and penetration signed distances with finite witness points.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_NEGATIVE_DISTANCE, sphere_capsule_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleSphere.LargeSphereSignedDistanceAlongCapsuleSide",
        "Checks the same large capsule-sphere signed-distance configurations through DART's single native signed-distance path.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_sphere1_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderSphereEdgeWitnessRegression",
        "Checks the wild cylinder-sphere edge-witness configuration as a near-boundary native distance robustness case with finite distance and witness points inside the cylinder and sphere geometry.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_box_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderBoxRegressionWitnessesInsideGeometry",
        "Checks the first wild cylinder-box witness configuration with finite signed distance, distance/witness consistency, and witness points inside the cylinder and box geometry.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_signed_distance.cpp",
        "GTEST_TEST(FCL_SIGNED_DISTANCE, cylinder_box_2)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderBoxRegressionWitnessesInsideGeometry",
        "Checks the second wild cylinder-box witness configuration with finite signed distance, distance/witness consistency, and witness points inside the cylinder and box geometry.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(SegmentSegmentNearestPtTest, BothZeroLength)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.DegenerateCenterLineEndpointsAndInterior",
        "Checks zero-length and near-zero capsule center lines through public signed-distance nearest-point recovery.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(SegmentSegmentNearestPtTest, OneZeroLength)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.DegenerateCenterLineEndpointsAndInterior",
        "Checks point-vs-segment endpoint and interior witness regions in both argument orders.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(SegmentSegmentNearestPtTest, ParallelSegments)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.ParallelCenterLineWitnessRegions",
        "Checks parallel center-line overlap, shared endpoint, lateral separation, and non-overlapping endpoint regions.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(SegmentSegmentNearestPtTest, NominalSegments)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.SkewCenterLineWitnessRegions",
        "Checks skew center-line intersection, interior-interior, endpoint-interior, and endpoint-endpoint witness regions.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(CapsuleCapsuleSegmentTest, NominalSeparatedCase)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.TransformedSignedDistanceConfigurations",
        "Checks transformed capsule poses with unique separated witness points and positive signed distance.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(CapsuleCapsuleSegmentTest, NominalIntersectingCase)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.TransformedSignedDistanceConfigurations",
        "Checks transformed capsule poses with unique penetrating witness points and negative signed distance.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(CapsuleCapsuleSegmentTest, SingleIntersectionCenterLines)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.TransformedSignedDistanceConfigurations",
        "Checks the maximum-penetration signed distance when capsule center lines intersect at one point.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_capsule.cpp",
        "TYPED_TEST(CapsuleCapsuleSegmentTest, OverlappingCenterLines)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleCapsule.TransformedSignedDistanceConfigurations",
        "Checks the maximum-penetration signed distance when capsule center lines overlap over an interval.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_box_1.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_box_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleBox.AxisAndEndpointWitnessCases",
        "Checks capsule-box side, top-face, and rotated endpoint distance cases with expected distances and witness coordinates or valid closest-feature intervals.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_box_1.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_box_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleBox.AxisAndEndpointWitnessCases",
        "Checks the same capsule-box distance cases through DART's single native distance path rather than upstream's solver-type split.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_box_2.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_box_ccd)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleBox.AxisAndEndpointWitnessCases",
        "Checks the offset rotated capsule endpoint behind a box with expected distance and unique witness coordinates.",
    ),
    (
        "FCL",
        "fcl/test/test_fcl_capsule_box_2.cpp",
        "GTEST_TEST(FCL_GEOMETRIC_SHAPES, distance_capsule_box_indep)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceCapsuleBox.AxisAndEndpointWitnessCases",
        "Checks the same offset rotated capsule-box endpoint distance case through DART's single native distance path rather than upstream's solver-type split.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_half_space_convex.cpp",
        "GTEST_TEST(HalfspaceConvexPrimitiveTest, CollisionTests)",
    ): (
        "covered",
        "tests/unit/collision/test_convex.cpp::ConvexCollision.PlaneTetrahedronHalfspaceConfigurations",
        "Checks separated and penetrating tetrahedron/halfspace configurations through canonical and embedded geometry frames, including transformed world frames and both pair orders.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, NearestPointInCylinder)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderSphere.SphereSeparatedAcrossFramesAndOrientations; tests/unit/collision/test_cylinder.cpp::CylinderSphere.SphereContactsAcrossFramesAndOrientations",
        "Ports nearest cap, barrel, edge, internal-center, and origin-nearest configurations through DART's public cylinder-sphere collision path.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, CollisionAcrossVaryingWorldFrames)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderSphere.SphereSeparatedAcrossFramesAndOrientations; tests/unit/collision/test_cylinder.cpp::CylinderSphere.SphereContactsAcrossFramesAndOrientations",
        "Runs the local configurations through identity and rotated/translated cylinder frames.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, CollisionWithSphereRotations)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderSphere.SphereSeparatedAcrossFramesAndOrientations; tests/unit/collision/test_cylinder.cpp::CylinderSphere.SphereContactsAcrossFramesAndOrientations",
        "Runs the same sphere-cylinder cases with non-identity sphere-frame rotations; sphere orientation must not affect DART results.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, CollisionIncompatibleScales)",
    ): (
        "covered",
        "tests/unit/collision/test_cylinder.cpp::CylinderSphere.ContactsWithIncompatibleScaleRatios",
        "Checks large-disk/tiny-sphere and tiny-disk/large-sphere face and barrel collision/separation cases across identity and transformed cylinder frames.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, DistanceAcrossVaryingWorldFrames)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderSphereSeparatedAcrossFramesAndOrientations",
        "Checks separated face, barrel, and edge nearest features across identity, translated, axis-rotated, and arbitrary cylinder frames.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, DistanceWithSphereRotations)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderSphereSeparatedAcrossFramesAndOrientations",
        "Runs the same cylinder-sphere distance cases with non-identity sphere orientations; sphere orientation must not affect native distance.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_cylinder.cpp",
        "GTEST_TEST(SphereCylinderPrimitiveTest, DistanceIncompatibleScales)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderSphereIncompatibleScaleSeparated",
        "Checks large-disk/tiny-sphere and tiny-disk/large-sphere separated nearest-point cases for face and barrel features.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, NearestPointInBox)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.SeparatedAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.SeparatedAcrossBoxFramesAndSphereOrientations",
        "Ports nearest face, vertex/corner, internal, boundary, and coincident-center box features through public collision and distance behavior.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, CollisionAcrossVaryingWorldFrames)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.SeparatedAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsAcrossBoxFramesAndSphereOrientations",
        "Checks sphere-box separated, external face/vertex, internal face, surface, and coincident-origin cases through identity and rotated/translated box frames.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, CollisionWithSphereRotations)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.SeparatedAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsAcrossBoxFramesAndSphereOrientations",
        "Runs the same sphere-box cases with non-identity sphere-frame rotations; sphere orientation must not affect DART results.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, CollisionIncompatibleScales)",
    ): (
        "covered",
        "tests/unit/collision/test_sphere_box.cpp::SphereBox.ContactsWithIncompatibleScaleRatios",
        "Checks long-skinny-box/small-sphere and tiny-box/large-sphere collision and separation cases.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, DistanceAcrossVaryingWorldFrames)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.SeparatedAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.ContactsSignedDistanceAcrossFramesAndOrientations",
        "Checks sphere-box distance configurations across identity, translated, rotated, arbitrary, and near-identity box frames; DART asserts signed penetration distance for colliding configurations.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, DistanceWithSphereRotations)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.SeparatedAcrossBoxFramesAndSphereOrientations; tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.ContactsSignedDistanceAcrossFramesAndOrientations",
        "Runs the same sphere-box distance configurations with identity, axis, arbitrary, and near-identity sphere orientations; sphere orientation must not affect DART distance results.",
    ),
    (
        "FCL",
        "fcl/test/narrowphase/detail/primitive_shape_algorithm/test_sphere_box.cpp",
        "GTEST_TEST(SphereBoxPrimitiveTest, DistanceIncompatibleScales)",
    ): (
        "covered",
        "tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.IncompatibleScaleSeparatedNearestPoints; tests/unit/collision/test_distance_core.cpp::DistanceSphereBox.IncompatibleScaleContactsSignedDistance",
        "Checks long-skinny-box/small-sphere and tiny-box/large-sphere separated nearest-point and colliding signed-distance cases.",
    ),
}


def classify_case(case: Case, scope: Scope) -> tuple[str, str, str]:
    override = CASE_MAP_OVERRIDES.get(
        (scope.project, case.source.as_posix(), case_display(case))
    )
    if override is not None:
        return override

    source = case.source.as_posix().lower()
    text = f"{case.suite} {case.name} {source}".lower()
    native_scope = scope.native_scope.lower()

    if "catalogued outside" in native_scope:
        return (
            "not-applicable",
            "outside native collision detector",
            "Accounted outside native collision unless a later pass finds direct collision behavior.",
        )

    if "benchmark superset" in native_scope:
        return (
            "new-benchmark-needed",
            "tests/benchmark/collision/",
            "Benchmark scenario needs a deterministic DART row.",
        )

    if "gpu/prototype" in native_scope:
        return (
            "fixture-needed",
            "future native GPU/prototype coverage",
            "Catalogue now; completion needs an explicit prototype gate decision.",
        )

    if scope.project == "FCL" and source == "fcl/test/test_fcl_math.cpp":
        return (
            "not-applicable",
            "FCL math and bounding-volume utilities",
            "This row targets FCL vector, Morton-code, and RSS bounding-volume helper APIs. DART native collision uses Eigen vectors plus native AABB/broadphase structures and does not expose those FCL utility contracts.",
        )

    if scope.project == "FCL" and source == "fcl/test/test_fcl_profiler.cpp":
        return (
            "not-applicable",
            "FCL profiler utility",
            "This row checks FCL's internal profiler helper, not DART native collision geometry, query, dispatch, or broadphase behavior.",
        )

    if (
        scope.project == "FCL"
        and source == "fcl/test/broadphase/test_broadphase_dynamic_aabb_tree.cpp"
    ):
        return (
            "covered",
            "tests/unit/collision/test_collision_world.cpp::CollisionWorld.UpdateDirtyById; tests/unit/collision/test_collision_world.cpp::CollisionWorld.CollideAllOrderingAndRepeatability; tests/unit/collision/test_collision_world.cpp::CollisionWorldBroadPhase.BuildSnapshotReusableBuffer",
            "Checks repeated native broadphase updates, deterministic pair ordering, and reusable broadphase snapshots.",
        )

    if scope.project == "FCL" and source.endswith(
        "test_fcl_broadphase_collision_1.cpp"
    ):
        if "dont_duplicate" in text:
            return (
                "covered",
                "tests/unit/collision/test_aabb_tree.cpp::AabbTreeBroadPhase.DeterministicPairOrder; tests/unit/collision/test_brute_force.cpp::BruteForceBroadPhase.DeterministicPairOrder; tests/unit/collision/test_spatial_hash.cpp::SpatialHashBroadPhase.DeterministicPairOrder; tests/unit/collision/test_sweep_and_prune.cpp::SweepAndPruneBroadPhase.DeterministicPairOrder",
                "Checks deterministic, unique broadphase pair reporting across native broadphase implementations.",
            )
        mesh_note = (
            " Mesh geometry is covered separately by native mesh collision and distance tests."
            if "mesh" in text
            else ""
        )
        return (
            "covered",
            "tests/unit/collision/test_aabb_tree.cpp::AabbTreeBroadPhase.BulkUpdateRange; tests/unit/collision/test_collision_world.cpp::CollisionWorld.UpdateDirtyById; tests/unit/collision/test_collision_world.cpp::CollisionWorld.CollideAllOrderingAndRepeatability",
            "Checks broadphase updates that turn separated objects into colliding pairs and preserve deterministic collision traversal."
            + mesh_note,
        )

    if scope.project == "FCL" and source.endswith(
        "test_fcl_broadphase_collision_2.cpp"
    ):
        if "empty" in text:
            return (
                "covered",
                "tests/unit/collision/test_collision_world.cpp::CollisionWorld.EmptyWorld; tests/unit/collision/test_aabb_tree.cpp::AabbTreeBroadPhase.QueryOverlapping_Empty; tests/unit/collision/test_brute_force.cpp::BruteForceBroadPhase.QueryOverlapping_Empty; tests/unit/collision/test_spatial_hash.cpp::SpatialHashBroadPhase.QueryOverlapping_Empty; tests/unit/collision/test_sweep_and_prune.cpp::SweepAndPruneBroadPhase.QueryOverlapping_Empty",
                "Checks empty native broadphase worlds and empty overlap queries.",
            )
        mesh_note = (
            " Mesh geometry is covered separately by native mesh collision and distance tests."
            if "mesh" in text
            else ""
        )
        return (
            "covered",
            "tests/unit/collision/test_aabb_tree.cpp::AabbTreeBroadPhase.ConsistentWithBruteForce; tests/unit/collision/test_spatial_hash.cpp::SpatialHashBroadPhase.ConsistentWithBruteForce; tests/unit/collision/test_sweep_and_prune.cpp::SweepAndPruneBroadPhase.ConsistentWithBruteForce; tests/unit/collision/test_collision_world.cpp::CollisionWorld.MultipleObjects",
            "Checks native broadphase collision and self-collision pair sets against brute force and world-level collision traversal."
            + mesh_note,
        )

    if scope.project == "FCL" and source.endswith("test_fcl_broadphase_distance.cpp"):
        mesh_target = (
            "; tests/unit/collision/test_mesh_mesh.cpp::MeshMesh.SingleTriangleAndLargeMesh"
            if "mesh" in text
            else ""
        )
        return (
            "covered",
            "tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.DistanceKeepsSearchingAfterPenetration; tests/unit/collision/test_collision_group.cpp::DartCollisionGroup.PersistentSceneTracksMovedObjects"
            + mesh_target,
            "Checks native group distance across multiple objects, self-distance/persistent-scene updates, and mesh distance where mesh rows are involved.",
        )

    if scope.project == "FCL" and source.startswith("fcl/test/geometry/shape/"):
        if source.endswith("test_box.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_shapes.cpp::BoxShape.Construction; tests/unit/collision/test_shapes.cpp::BoxShape.ComputeLocalAabb",
                "Checks native box type, half extents, and local bounds; DART native does not expose a string-representation API.",
            )

        if source.endswith("test_sphere.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_shapes.cpp::SphereShape.Construction; tests/unit/collision/test_shapes.cpp::SphereShape.ComputeLocalAabb",
                "Checks native sphere type, radius, and local bounds; DART native does not expose a string-representation API.",
            )

        if source.endswith("test_cylinder.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_shapes.cpp::CylinderShape.Construction; tests/unit/collision/test_shapes.cpp::CylinderShape.ComputeLocalAabb",
                "Checks native cylinder type, radius/height, and local bounds; DART native does not expose a string-representation API.",
            )

        if source.endswith("test_capsule.cpp"):
            if any(
                token in text for token in ("volume", "centerofmass", "momentofinertia")
            ):
                return (
                    "not-applicable",
                    "native collision shape API",
                    "DART native collision shapes expose query geometry and local bounds, not volume, center-of-mass, or inertia APIs.",
                )
            return (
                "covered",
                "tests/unit/collision/test_shapes.cpp::CapsuleShape.Construction; tests/unit/collision/test_shapes.cpp::CapsuleShape.ComputeLocalAabbAcrossRadiusHeightCases",
                "Checks native capsule type, radius/height, and local bounds across radius/height aspect ratios; DART native does not expose a string-representation API.",
            )

        if source.endswith("test_plane.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_plane.cpp::PlaneShape.Construction; tests/unit/collision/test_plane.cpp::PlaneShape.NormalNormalization; tests/unit/collision/test_plane.cpp::PlaneShape.LocalAabbIsUnbounded",
                "Checks native plane type, normalized normal, offset, and unbounded local bounds; DART native does not expose a string-representation API.",
            )

        if source.endswith("test_halfspace.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_plane.cpp::PlaneShape.Construction; tests/unit/collision/test_plane.cpp::PlaneShape.NormalNormalization; tests/unit/collision/test_plane.cpp::PlaneShape.LocalAabbIsUnbounded",
                "DART native represents half-space collision queries with PlaneShape; checks normal/offset storage and unbounded local bounds.",
            )

        if source.endswith("test_cone.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter",
                "Checks DART cone adaptation into a native convex envelope with expected vertex count and local bounds.",
            )

        if source.endswith("test_ellipsoid.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.EllipsoidShapeAdapter; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.SphericalEllipsoidShapeAdapter",
                "Checks DART ellipsoid adaptation into native convex or sphere geometry with expected local bounds.",
            )

        if source.endswith("test_convex.cpp"):
            if any(
                token in text for token in ("volume", "centerofmass", "momentofinertia")
            ):
                return (
                    "not-applicable",
                    "native collision shape API",
                    "DART native ConvexShape is a vertex support shape for collision queries; it does not expose polytope mass-property APIs.",
                )
            if "watertightvalidation" in text or "useedgewalkingconditions" in text:
                return (
                    "not-applicable",
                    "native collision shape API",
                    "DART native ConvexShape stores vertices for support queries and does not expose face topology validation or edge-walking state.",
                )
            if "localaabbcomputation" in text:
                return (
                    "covered",
                    "tests/unit/collision/test_shapes.cpp::ConvexShape.ScaledPolytopeLocalBounds",
                    "Checks native convex local bounds for scaled cube and tetrahedron vertex clouds.",
                )
            if "supportvertex_tetrahedron" in text:
                return (
                    "covered",
                    "tests/unit/collision/test_shapes.cpp::ConvexShape.TetrahedronSupportVerticesAcrossScales",
                    "Checks native convex support vertices for centered tetrahedra across small, unit, and large scales.",
                )
            return (
                "covered",
                "tests/unit/collision/test_shapes.cpp::ConvexShape.Construction; tests/unit/collision/test_shapes.cpp::ConvexShape.SupportMatchesExhaustiveVertexSearch",
                "Checks native convex construction, vertex storage, and support-query behavior; DART native does not expose a string-representation API.",
            )

    if scope.project == "FCL" and source == "fcl/test/test_fcl_geometric_shapes.cpp":
        if case.name in {
            "shapeIntersection_conecone",
            "shapeIntersectionGJK_conecone",
        }:
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexPairsCollideAcrossPairOrder; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter",
                "DART adapts ConeShape to a native convex shape. The public dispatcher checks cone-cone collision in both pair orders with finite contact data; the adapter test checks the cone envelope used by native collision.",
            )

        if case.name in {
            "shapeIntersection_cylindercone",
            "shapeIntersectionGJK_cylindercone",
        }:
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexPairsCollideAcrossPairOrder; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter",
                "Checks cylinder-cone collision in both pair orders through the public native dispatcher. ConeShape is represented by the native convex adapter; CylinderShape remains a native cylinder primitive.",
            )

        if case.name in {
            "shapeIntersection_ellipsoidellipsoid",
            "shapeIntersectionGJK_ellipsoidellipsoid",
        }:
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexPairsCollideAcrossPairOrder; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.EllipsoidShapeAdapter; tests/unit/collision/test_dart_collide.cpp::DARTCollide.MainDispatcherEllipsoidEllipsoid",
                "DART adapts non-spherical EllipsoidShape to a native convex shape. The public dispatcher checks ellipsoid-ellipsoid collision in both pair orders, and existing dispatcher coverage checks a direct ellipsoid pair.",
            )

        if case.name in {
            "shapeDistance_cylindercylinder",
            "shapeDistanceGJK_cylindercylinder",
        }:
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexDistancesAcrossPairOrder; tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderCylinderThinHeight",
                "Checks positive cylinder-cylinder native distance in both pair orders and a thin-height cylinder distance regression.",
            )

        if case.name in {"shapeDistance_conecone", "shapeDistanceGJK_conecone"}:
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexDistancesAcrossPairOrder; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter",
                "Checks positive cone-cone distance in both pair orders through the public native dispatcher using DART's convex cone adapter.",
            )

        if case.name == "shapeDistance_conecylinder":
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexDistancesAcrossPairOrder; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.ConeShapeAdapter",
                "Checks positive cylinder-cone distance in both pair orders through the public native dispatcher.",
            )

        if case.name in {
            "shapeDistance_ellipsoidellipsoid",
            "shapeDistanceGJK_ellipsoidellipsoid",
        }:
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexDistancesAcrossPairOrder; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.EllipsoidShapeAdapter; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.SphericalEllipsoidShapeAdapter",
                "Checks positive ellipsoid-ellipsoid distance in both pair orders through DART's convex ellipsoid adapter, plus adapter rows for spherical and non-spherical ellipsoid representations.",
            )

        if case.name == "reversibleShapeIntersection_allshapes":
            return (
                "covered",
                "tests/unit/collision/test_narrow_phase.cpp::NarrowPhase.IsSupported; tests/unit/collision/test_dart_collision_detector.cpp::DartCollisionDetector.SupportedShapePairs; tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexPairsCollideAcrossPairOrder; tests/unit/collision/test_plane.cpp::PlaneShape.HalfspacePrimitiveContactsAcrossTransforms",
                "Checks native support-matrix symmetry and public pair-order collision for primitive and adapted-convex rows. Plane/halfspace pair-order behavior is covered by the plane contact suite.",
            )

        if case.name == "reversibleShapeDistance_allshapes":
            return (
                "covered",
                "tests/unit/collision/test_collision_backend.cpp::CollisionBackend.PrimitiveAndAdaptedConvexDistancesAcrossPairOrder; tests/unit/collision/test_distance_core.cpp::NarrowPhaseDistance.CylinderSphereSeparatedAcrossFramesAndOrientations",
                "Checks public native distance symmetry by comparing distances and swapped witness points across pair order for cylinder, cone, and ellipsoid rows; primitive sphere/capsule and cylinder/sphere distance rows are covered separately.",
            )

    if scope.project == "FCL" and source.startswith(
        "fcl/test/narrowphase/detail/convexity_based_algorithm/"
    ):
        if source.endswith("test_gjk_libccd-inl_gjk_initializer.cpp"):
            return (
                "covered",
                "tests/unit/collision/test_gjk.cpp::Gjk.TransformedConvexSupportMatchesVertexSearch",
                "Checks transformed native convex support points against exhaustive world-space vertex search.",
            )
        if source.endswith(
            (
                "test_gjk_libccd-inl_epa.cpp",
                "test_gjk_libccd-inl_extractclosestpoints.cpp",
                "test_gjk_libccd-inl_gjk_dosimplex2.cpp",
            )
        ):
            return (
                "not-applicable",
                "dart/collision/native/narrow_phase/gjk.cpp private helpers",
                "This upstream row targets implementation-private simplex/EPA helper behavior. DART exposes public GJK/EPA/MPR query results instead, covered by test_gjk.cpp, test_gjk_degenerate.cpp, and test_libccd_algorithms.cpp.",
            )

    if "geometry/shape" in source or "/shape/" in source:
        return (
            "mapping-needed",
            "tests/unit/collision/test_shapes.cpp; tests/unit/collision/test_convex.cpp",
            "Shape geometry/support expectation needs a DART-native equivalent or non-applicability reason.",
        )

    if any(token in text for token in ("gjk", "epa", "simplex", "libccd", "mpr")):
        return (
            "mapping-needed",
            "tests/unit/collision/test_gjk.cpp; test_gjk_degenerate.cpp; test_libccd_algorithms.cpp",
            "Algorithm detail case needs exact simplex/GJK/EPA/MPR mapping.",
        )

    if "broadphase" in text or "broad_phase" in text or "aabb" in text:
        return (
            "mapping-needed",
            "tests/unit/collision/test_aabb_tree.cpp; test_brute_force.cpp; test_spatial_hash.cpp",
            "Broadphase case needs DART broadphase mapping.",
        )

    if any(
        token in text
        for token in (
            "sphere",
            "box",
            "capsule",
            "cylinder",
            "half",
            "plane",
            "mesh",
            "trimesh",
            "heightfield",
            "ray",
            "convex",
            "point_depth",
        )
    ):
        return (
            "mapping-needed",
            "tests/unit/collision narrowphase/raycast tests",
            "Primitive/query case needs file-by-file DART mapping.",
        )

    return (
        "mapping-needed",
        "native collision test suite",
        "Needs manual classification.",
    )


def render_case_map(source_root: Path, inventory: list[ScopeInventory]) -> str:
    lines = [
        "# Upstream Collision Case Map",
        "",
        "<!-- Generated by scripts/inventory_upstream_collision_coverage.py; do not edit by hand. -->",
        "",
        f"Generated: `{datetime.now().astimezone().isoformat(timespec='seconds')}`",
        "",
        "## Sources",
        "",
        *source_table(source_root),
        "",
        "## Summary",
        "",
        "| Status | Rows |",
        "| --- | ---: |",
    ]

    row_data = []
    for item in inventory:
        for case in item.cases:
            status, target, note = classify_case(case, item.scope)
            row_data.append((status, item, case, target, note))
        for benchmark in item.benchmark_cases:
            row_data.append(
                (
                    "new-benchmark-needed",
                    item,
                    benchmark,
                    "tests/benchmark/collision/",
                    "Benchmark scenario needs a deterministic DART row.",
                )
            )

    counts: dict[str, int] = {}
    for status, *_ in row_data:
        counts[status] = counts.get(status, 0) + 1
    for status in sorted(counts):
        lines.append(f"| `{status}` | {counts[status]} |")

    lines.extend(
        [
            "",
            "## Rows",
            "",
            "| Status | Project | Scope | Upstream row | Source | DART target | Notes |",
            "| --- | --- | --- | --- | --- | --- | --- |",
        ]
    )

    for status, item, row, target, note in row_data:
        if isinstance(row, Case):
            row_name = f"`{case_display(row)}`"
            source = f"`{row.source.as_posix()}`"
        else:
            row_name = row
            source = f"`{item.scope.path}`"
        lines.append(
            "| "
            f"`{status}` | {item.scope.project} | {item.scope.name} | "
            f"{row_name} | {source} | {target} | {note} |"
        )

    lines.append("")
    return "\n".join(lines)


def render(source_root: Path, inventory: list[ScopeInventory]) -> str:
    lines = [
        "# Upstream Collision Coverage Inventory",
        "",
        "<!-- Generated by scripts/inventory_upstream_collision_coverage.py; do not edit by hand. -->",
        "",
        f"Generated: `{datetime.now().astimezone().isoformat(timespec='seconds')}`",
        "",
        "## Sources",
        "",
        *source_table(source_root),
        "",
        "## Inventory Policy",
        "",
        "- This is a working inventory for a dev task, not durable project state.",
        "- FCL's test tree is treated as collision-library coverage.",
        "- Bullet and ODE contain non-collision tests; those are still catalogued",
        "  so the task can prove they were considered instead of silently ignored.",
        "- Native-collision completion requires every in-scope upstream",
        "  correctness case and benchmark scenario to map to a DART test,",
        "  benchmark, fixture, or explicit non-applicability decision.",
        "",
        "## Summary",
        "",
        "| Project | Scope | Native-collision scope | Source files | Extracted cases | Benchmark cases |",
        "| --- | --- | --- | ---: | ---: | ---: |",
    ]
    for item in inventory:
        lines.append(
            "| "
            f"{item.scope.project} | {item.scope.name} | {item.scope.native_scope} | "
            f"{len(item.files)} | {len(item.cases)} | {len(item.benchmark_cases)} |"
        )

    for item in inventory:
        lines.extend(
            [
                "",
                f"## {item.scope.project}: {item.scope.name}",
                "",
                f"Role: {item.scope.role}",
                "",
                f"Native-collision scope: {item.scope.native_scope}",
                "",
                "### Source Files",
                "",
            ]
        )
        if item.files:
            for path in item.files:
                lines.append(f"- `{format_path(path, source_root)}`")
        else:
            lines.append("- None found.")

        if item.benchmark_cases:
            lines.extend(["", "### Benchmark Cases", ""])
            for case in item.benchmark_cases:
                lines.append(f"- {case}")

        if item.cases:
            lines.extend(["", "### Extracted Test Cases", ""])
            for case in item.cases:
                lines.append(
                    f"- `{case_display(case)}` from `{case.source.as_posix()}`"
                )

    lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    inventory = inventories(args.source_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render(args.source_root, inventory))
    print(f"Wrote {args.output}")
    args.case_map_output.parent.mkdir(parents=True, exist_ok=True)
    args.case_map_output.write_text(render_case_map(args.source_root, inventory))
    print(f"Wrote {args.case_map_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
