from __future__ import annotations

import importlib
import math
import os
from pathlib import Path

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
    if not hasattr(module, "World"):
        if _cache_reports_experimental_disabled():
            pytest.skip("DART_BUILD_SIMULATION_EXPERIMENTAL is disabled")
        raise AssertionError(
            "dartpy.simulation_experimental imported but did not expose "
            "the experimental World binding"
        )
    return module


def _translation_transform(x: float, y: float, z: float):
    return (
        (1.0, 0.0, 0.0, x),
        (0.0, 1.0, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def test_experimental_world_module_is_separate_from_legacy_simulation():
    sx = _simulation_experimental()

    assert dart.simulation_experimental is sx
    assert sx is not dart.simulation
    assert not hasattr(dart, "next")


def test_experimental_world_smoke():
    sx = _simulation_experimental()

    world = sx.World()
    assert not world.is_simulation_mode
    assert world.num_multi_bodies == 0
    assert world.num_rigid_bodies == 0

    multi_body = world.add_multi_body("robot")
    assert multi_body.name == "robot"
    assert multi_body.num_links == 0
    assert multi_body.num_joints == 0
    assert multi_body.num_dofs == 0

    multi_body.name = "renamed_robot"
    assert multi_body.name == "renamed_robot"
    assert world.get_multi_body("renamed_robot").name == "renamed_robot"
    assert world.get_multi_body("missing") is None
    assert world.num_multi_bodies == 1

    rigid_body = world.add_rigid_body("box")
    assert rigid_body.name == "box"
    assert world.has_rigid_body("box")
    assert not world.has_rigid_body("missing")
    assert world.num_rigid_bodies == 1

    world.enter_simulation_mode()
    assert world.is_simulation_mode
    world.update_kinematics()

    world.clear()
    assert not world.is_simulation_mode
    assert world.num_multi_bodies == 0
    assert world.num_rigid_bodies == 0


def test_experimental_multibody_link_joint_common_path():
    sx = _simulation_experimental()

    world = sx.World()
    arm = world.add_multi_body("arm")

    base = arm.add_link("base")
    assert base.name == "base"
    assert base.is_valid
    assert not base.parent_joint.is_valid
    assert base.translation.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert base.quaternion.tolist() == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert arm.num_links == 1
    assert arm.num_joints == 0
    assert arm.num_dofs == 0

    spec = sx.JointSpec(
        name="elbow",
        type=sx.JointType.REVOLUTE,
        axis=(0.0, 0.0, 2.0),
    )
    forearm = arm.add_link("forearm", parent=base, joint=spec)

    assert forearm.name == "forearm"
    assert forearm.get_name() == "forearm"
    assert arm.num_links == 2
    assert arm.num_joints == 1
    assert arm.num_dofs == 1
    assert arm.get_link("base").name == "base"
    assert arm.get_link("missing") is None

    joint = forearm.parent_joint
    assert joint.is_valid
    assert joint.name == "elbow"
    assert joint.get_name() == "elbow"
    assert joint.type == sx.JointType.REVOLUTE
    assert joint.get_type() == sx.JointType.REVOLUTE
    assert joint.axis.tolist() == pytest.approx([0.0, 0.0, 1.0])
    assert joint.get_axis().tolist() == pytest.approx([0.0, 0.0, 1.0])
    assert joint.parent_link.name == "base"
    assert joint.get_parent_link().name == "base"
    assert joint.child_link.name == "forearm"
    assert joint.get_child_link().name == "forearm"
    assert arm.get_joint("elbow").child_link.name == "forearm"
    assert arm.get_joint("missing") is None

    world.step()

    assert world.is_simulation_mode
    assert forearm.translation.tolist() == pytest.approx([0.0, 0.0, 0.0])


def test_experimental_frame_handles_support_kinematics_only_updates():
    sx = _simulation_experimental()

    world = sx.World()
    parent = world.add_free_frame("parent")
    child = world.add_fixed_frame(
        "sensor",
        parent,
        offset=_translation_transform(0.0, 2.0, 0.0),
    )

    assert isinstance(parent, sx.Frame)
    assert isinstance(parent, sx.FreeFrame)
    assert isinstance(child, sx.Frame)
    assert isinstance(child, sx.FixedFrame)
    assert sx.Frame.world().is_world
    assert parent.name == "parent"
    assert child.name == "sensor"
    assert child.parent_frame == parent

    world.enter_simulation_mode()
    assert child.translation.tolist() == pytest.approx([0.0, 2.0, 0.0])

    parent.local_transform = _translation_transform(3.0, 0.0, 0.0)

    assert child.translation.tolist() == pytest.approx([3.0, 2.0, 0.0])
    world.update_kinematics()
    assert child.transform[:3, 3].tolist() == pytest.approx([3.0, 2.0, 0.0])


def test_experimental_world_common_path_properties_and_step_count():
    sx = _simulation_experimental()

    world = sx.World(time_step=0.01)
    assert world.time_step == pytest.approx(0.01)
    assert world.time == pytest.approx(0.0)
    assert world.frame == 0
    assert not world.is_simulation_mode

    box = world.add_rigid_body(
        "box",
        mass=2.0,
        position=(1.0, 2.0, 3.0),
        linear_velocity=(1.0, 0.0, 0.0),
    )

    assert box.name == "box"
    assert box.translation.tolist() == pytest.approx([1.0, 2.0, 3.0])
    assert box.quaternion.tolist() == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert world.has_rigid_body("box")
    assert world.get_rigid_body("box") == box
    assert world.getRigidBody("box").name == "box"
    assert world.get_rigid_body("missing") is None

    world.step(n=0)
    assert not world.is_simulation_mode
    assert world.time == pytest.approx(0.0)
    assert world.frame == 0

    world.step(n=3)

    assert world.is_simulation_mode
    assert world.time == pytest.approx(0.03)
    assert world.frame == 3
    assert box.translation.tolist() == pytest.approx([1.03, 2.0, 3.0])


def test_experimental_rigid_body_options_value_object():
    sx = _simulation_experimental()

    options = sx.RigidBodyOptions(
        mass=3.0,
        position=(0.0, 0.5, 1.0),
        orientation=(1.0, 0.0, 0.0, 0.0),
        linear_velocity=(0.1, 0.2, 0.3),
        angular_velocity=(0.4, 0.5, 0.6),
    )

    assert options.mass == pytest.approx(3.0)
    assert options.position.tolist() == pytest.approx([0.0, 0.5, 1.0])
    assert options.orientation.tolist() == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert options.linear_velocity.tolist() == pytest.approx([0.1, 0.2, 0.3])
    assert options.angular_velocity.tolist() == pytest.approx([0.4, 0.5, 0.6])

    options.position = (2.0, 3.0, 4.0)
    options.linear_velocity = (1.0, 0.0, 0.0)

    world = sx.World(time_step=0.1)
    box = world.add_rigid_body("box", options)
    world.step()

    assert box.translation.tolist() == pytest.approx([2.1, 3.0, 4.0])


def test_experimental_rigid_body_options_reject_invalid_values():
    sx = _simulation_experimental()

    with pytest.raises(Exception, match="mass must be positive and finite"):
        sx.RigidBodyOptions(mass=0.0)

    with pytest.raises(
        Exception, match="inertia must be symmetric positive definite"
    ):
        sx.RigidBodyOptions(
            inertia=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
        )

    with pytest.raises(Exception, match="position must contain only finite"):
        sx.RigidBodyOptions(position=(math.inf, 0.0, 0.0))

    with pytest.raises(Exception, match="orientation must be finite and non-zero"):
        sx.RigidBodyOptions(orientation=(0.0, 0.0, 0.0, 0.0))

    options = sx.RigidBodyOptions()

    with pytest.raises(Exception, match="linearVelocity must contain only finite"):
        options.linear_velocity = (math.nan, 0.0, 0.0)

    with pytest.raises(Exception, match="angularVelocity must contain only finite"):
        options.angular_velocity = (0.0, math.inf, 0.0)

    world = sx.World()
    with pytest.raises(Exception, match="mass must be positive and finite"):
        world.add_rigid_body("box", mass=math.inf)
