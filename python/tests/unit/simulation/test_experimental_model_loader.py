from __future__ import annotations

import importlib
import os
from pathlib import Path

import numpy as np
import pytest

import dartpy as dart


def _cache_reports_experimental_disabled() -> bool:
    override = os.environ.get("DART_BUILD_SIMULATION_EXPERIMENTAL_OVERRIDE")
    if override and override.lower() in {"0", "false", "no", "off"}:
        return True

    repo_root = Path(__file__).resolve().parents[4]
    build_type = (
        os.environ.get("BUILD_TYPE")
        or os.environ.get("CMAKE_BUILD_TYPE")
        or "Release"
    )
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    candidates: list[Path] = []
    if os.environ.get("CMAKE_BUILD_DIR"):
        candidates.append(Path(os.environ["CMAKE_BUILD_DIR"]) / "CMakeCache.txt")
    candidates.extend(
        [
            repo_root / "build" / pixi_env / "cpp" / build_type / "CMakeCache.txt",
            repo_root / "build" / pixi_env / "cpp" / "CMakeCache.txt",
        ]
    )

    for cache in candidates:
        if not cache.is_file():
            continue
        for line in cache.read_text(encoding="utf-8").splitlines():
            if not line.startswith("DART_BUILD_SIMULATION_EXPERIMENTAL:"):
                continue
            return line.rsplit("=", 1)[-1].strip().lower() in {
                "0",
                "false",
                "no",
                "off",
            }
    return False


def _simulation_experimental():
    try:
        module = importlib.import_module("dartpy.simulation_experimental")
    except ModuleNotFoundError as exc:
        if _cache_reports_experimental_disabled():
            pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
        raise AssertionError(
            "dartpy.simulation_experimental should be available when "
            "dart-simulation-experimental is built"
        ) from exc
    if not hasattr(module, "build_multibody_from_skeleton"):
        if _cache_reports_experimental_disabled():
            pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
        raise AssertionError(
            "dartpy.simulation_experimental imported but did not expose "
            "build_multibody_from_skeleton"
        )
    return module


def _two_link_revolute_skeleton():
    """A revolute-revolute serial chain whose root joint sits at the origin and
    whose second joint is offset one link down (matches the C++ parity test)."""
    skeleton = dart.Skeleton("double_pendulum")

    joint1, body1 = skeleton.create_revolute_joint_and_body_node_pair()
    joint1.set_axis(np.array([0.0, 1.0, 0.0]))
    body1.set_inertia(
        dart.Inertia(2.0, [0.0, 0.0, -0.5], np.diag([0.05, 0.05, 0.02]))
    )

    joint2, body2 = skeleton.create_revolute_joint_and_body_node_pair(body1)
    joint2.set_axis(np.array([0.0, 1.0, 0.0]))
    offset = np.identity(4)
    offset[2, 3] = -1.0
    joint2.set_transform_from_parent_body_node(offset)
    body2.set_inertia(
        dart.Inertia(1.0, [0.0, 0.0, -0.5], np.diag([0.03, 0.03, 0.01]))
    )

    # The legacy skeleton and the experimental World share the same default
    # gravity (0, 0, -9.81), so no explicit gravity is set here.
    return skeleton


def test_build_multibody_from_skeleton_matches_legacy():
    sx = _simulation_experimental()
    skeleton = _two_link_revolute_skeleton()
    skeleton.set_positions([0.3, -0.5])
    skeleton.set_velocities([0.7, 0.2])

    legacy_mass = np.array(skeleton.get_mass_matrix())
    legacy_coriolis_gravity = np.array(skeleton.get_coriolis_and_gravity_forces())

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    # One extra link: the synthetic world base.
    assert multibody.num_links == skeleton.get_num_body_nodes() + 1
    assert multibody.num_joints == skeleton.get_num_body_nodes()
    assert multibody.num_dofs == skeleton.get_num_dofs() == 2

    # DART 6 parity on the bound dynamics accessors.
    assert np.allclose(np.array(multibody.mass_matrix), legacy_mass, atol=1e-9)
    assert np.allclose(
        np.array(multibody.coriolis_and_gravity_forces),
        legacy_coriolis_gravity,
        atol=1e-9,
    )


def test_build_multibody_options_and_copy_state():
    sx = _simulation_experimental()
    skeleton = _two_link_revolute_skeleton()
    skeleton.set_positions([0.4, 0.25])

    world = sx.World()
    options = sx.SkeletonToMultibodyOptions()
    options.name = "loaded_robot"
    options.base_link_name = "ground"
    options.copy_state = True
    multibody = sx.build_multibody_from_skeleton(world, skeleton, options)

    assert multibody.name == "loaded_robot"
    assert multibody.get_link("ground") is not None
    joint_positions = [joint.position[0] for joint in multibody.joints]
    assert np.allclose(joint_positions, [0.4, 0.25], atol=1e-12)

    # With copy_state disabled, the multibody stays at the zero configuration.
    world_zero = sx.World()
    options_zero = sx.SkeletonToMultibodyOptions()
    options_zero.copy_state = False
    multibody_zero = sx.build_multibody_from_skeleton(
        world_zero, skeleton, options_zero
    )
    zero_positions = [joint.position[0] for joint in multibody_zero.joints]
    assert np.allclose(zero_positions, [0.0, 0.0], atol=1e-12)


def test_build_multibody_steps_under_gravity():
    sx = _simulation_experimental()
    skeleton = _two_link_revolute_skeleton()
    # Start off the hanging equilibrium so gravity produces a non-zero torque.
    skeleton.set_positions([0.6, 0.3])

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    world.time_step = 1e-3
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    world.step(50)
    velocities = [abs(joint.velocity[0]) for joint in multibody.joints]
    assert max(velocities) > 0.0


def test_build_multibody_from_multi_tree_skeleton():
    sx = _simulation_experimental()

    # A skeleton with two independent root bodies (two trees). Both attach
    # beneath the single synthetic base link.
    skeleton = dart.Skeleton("two_trees")
    for _ in range(2):
        joint, body = skeleton.create_revolute_joint_and_body_node_pair()
        joint.set_axis(np.array([0.0, 1.0, 0.0]))
        body.set_inertia(
            dart.Inertia(1.0, [0.0, 0.0, -0.5], np.diag([0.02, 0.02, 0.01]))
        )
    skeleton.set_positions([0.2, -0.3])
    skeleton.set_velocities([0.4, 0.1])

    legacy_mass = np.array(skeleton.get_mass_matrix())
    legacy_coriolis_gravity = np.array(skeleton.get_coriolis_and_gravity_forces())

    world = sx.World()
    world.gravity = [0.0, 0.0, -9.81]
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_links == skeleton.get_num_body_nodes() + 1
    assert multibody.num_dofs == 2
    assert np.allclose(np.array(multibody.mass_matrix), legacy_mass, atol=1e-9)
    assert np.allclose(
        np.array(multibody.coriolis_and_gravity_forces),
        legacy_coriolis_gravity,
        atol=1e-9,
    )


def test_build_multibody_from_urdf_skeleton():
    sx = _simulation_experimental()

    loader = dart.io.DartLoader()
    options = dart.io.DartLoaderOptions()
    options.m_default_root_joint_type = dart.io.DartLoaderRootJointType.Fixed
    loader.set_options(options)
    skeleton = loader.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    assert skeleton is not None

    world = sx.World()
    multibody = sx.build_multibody_from_skeleton(world, skeleton)

    assert multibody.num_dofs == skeleton.get_num_dofs()
    assert multibody.num_links == skeleton.get_num_body_nodes() + 1
    # A converted, fixed-base arm steps without error.
    world.step(1)


def test_build_multibody_rejects_unsupported_joint():
    sx = _simulation_experimental()

    skeleton = dart.Skeleton("ball")
    _, body = skeleton.create_ball_joint_and_body_node_pair()
    body.set_inertia(
        dart.Inertia(1.0, [0.0, 0.0, 0.0], np.diag([0.01, 0.01, 0.01]))
    )

    world = sx.World()
    with pytest.raises(Exception, match="does not yet support"):
        sx.build_multibody_from_skeleton(world, skeleton)
