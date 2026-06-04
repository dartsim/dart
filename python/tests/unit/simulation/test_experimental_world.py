from __future__ import annotations

import gc
import importlib
import math
import os
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest


def _cache_reports_experimental_disabled() -> bool:
    override = os.environ.get("DART_BUILD_SIMULATION_EXPERIMENTAL_OVERRIDE")
    if override and override.lower() in {"0", "false", "no", "off"}:
        return True

    repo_root = Path(__file__).resolve().parents[4]
    build_type = (
        os.environ.get("BUILD_TYPE") or os.environ.get("CMAKE_BUILD_TYPE") or "Release"
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


def test_experimental_api_exposes_python_names_only():
    sx = _simulation_experimental()

    forbidden_names = {
        sx.Frame: (
            "get_name",
            "get_parent_frame",
            "set_parent_frame",
            "get_local_transform",
            "get_transform",
            "is_valid_handle",
            "getName",
            "getParentFrame",
            "setParentFrame",
            "getLocalTransform",
            "getTransform",
            "isValid",
            "isWorld",
        ),
        sx.Multibody: (
            "addLink",
            "getLink",
            "getJoint",
            "getLinks",
            "getJoints",
            "getLinkNames",
            "getJointNames",
            "getLinkCount",
            "getJointCount",
            "getDOFCount",
            "isValid",
            "get_links",
            "get_joints",
        ),
        sx.RigidBody: (
            "get_name",
            "set_transform",
            "get_linear_velocity",
            "set_linear_velocity",
            "get_angular_velocity",
            "set_angular_velocity",
            "get_mass",
            "set_mass",
            "get_inertia",
            "set_inertia",
            "get_force",
            "set_force",
            "get_torque",
            "set_torque",
            "setTransform",
            "getLinearVelocity",
            "setLinearVelocity",
            "getAngularVelocity",
            "setAngularVelocity",
            "getMass",
            "setMass",
            "getInertia",
            "setInertia",
            "getForce",
            "setForce",
            "applyForce",
            "clearForce",
            "getTorque",
            "setTorque",
            "applyTorque",
            "clearTorque",
        ),
        sx.Joint: (
            "get_name",
            "get_type",
            "get_axis",
            "get_num_dofs",
            "get_position",
            "set_position",
            "get_velocity",
            "set_velocity",
            "get_parent_link",
            "get_child_link",
            "get_parent_rigid_body",
            "get_child_rigid_body",
            "getName",
            "getType",
            "getAxis",
            "getDOFCount",
            "getPosition",
            "setPosition",
            "getVelocity",
            "setVelocity",
            "getParentLink",
            "getChildLink",
            "getParentRigidBody",
            "getChildRigidBody",
            "isValid",
        ),
        sx.World: (
            "addMultibody",
            "getMultibody",
            "hasMultibody",
            "getMultibodyCount",
            "addRigidBody",
            "addRigidBodyFixedJoint",
            "getRigidBodyFixedJoint",
            "hasRigidBodyFixedJoint",
            "getRigidBodyFixedJoints",
            "getRigidBodyFixedJointCount",
            "getRigidBody",
            "hasRigidBody",
            "getRigidBodyCount",
            "addLoopClosure",
            "getLoopClosure",
            "hasLoopClosure",
            "getLoopClosureCount",
            "get_multibody_count",
            "get_loop_closure_count",
            "get_rigid_body_count",
            "updateKinematics",
            "enterSimulationMode",
        ),
        sx.LoopClosure: (
            "get_name",
            "get_family",
            "get_frame_a",
            "get_frame_b",
            "get_offset_a",
            "get_offset_b",
            "get_runtime_policy",
            "set_runtime_policy",
            "getName",
            "getFamily",
            "getFrameA",
            "getFrameB",
            "getOffsetA",
            "getOffsetB",
            "getRuntimePolicy",
            "setRuntimePolicy",
            "computeResidual",
            "isValid",
        ),
    }

    for target, names in forbidden_names.items():
        for name in names:
            assert not hasattr(target, name), f"{target.__name__}.{name}"

    assert not hasattr(sx.Link, "get_name")
    assert not hasattr(sx.Link, "get_parent_joint")
    assert not hasattr(sx.FreeFrame, "get_local_transform")
    assert not hasattr(sx.FreeFrame, "set_local_transform")
    assert not hasattr(sx.FixedFrame, "get_local_transform")
    assert not hasattr(sx.FixedFrame, "set_local_transform")


def test_experimental_stub_tracks_public_runtime_symbols():
    sx = _simulation_experimental()
    repo_root = Path(__file__).resolve().parents[4]
    stub = (
        repo_root / "python" / "stubs" / "dartpy" / "simulation_experimental.pyi"
    ).read_text(encoding="utf-8")

    public_symbols = (
        "WorldSyncStage",
        "ClosureKinematicsPolicy",
        "ClosureDynamicsPolicy",
        "LoopClosureRuntimePolicy",
        "LoopClosureResidual",
        "LoopClosureResidualCoordinates",
        "SkeletonLoadOptions",
        "ModelFormat",
        "RootJointType",
        "ReadOptions",
        "StateSpace",
        "StateVariable",
    )
    for symbol in public_symbols:
        assert hasattr(sx, symbol), symbol
        assert f'"{symbol}"' in stub
        assert f"class {symbol}" in stub

    assert hasattr(sx, "add_skeleton")
    assert '"add_skeleton"' in stub
    assert "def add_skeleton(" in stub
    assert hasattr(sx, "add_world")
    assert '"add_world"' in stub
    assert "def add_world(" in stub

    for member in (
        "runtime_policy",
        "enabled",
        "kinematics",
        "dynamics",
        "compute_residual",
        "def sync(",
        "force_available",
        "add_variable",
        "variable_names",
        "link_names",
        "joint_names",
        "shape_index_a",
        "local_point_a",
        "has_multibody",
        "has_rigid_body_fixed_joint",
        "num_rigid_body_fixed_joints",
        "is_valid",
    ):
        assert member in stub

    forbidden_stub_members = (
        "def get_name(",
        "def set_transform(",
        "def get_position(",
        "def set_position(",
        "def get_runtime_policy(",
        "def set_runtime_policy(",
        "def getMultibody(",
        "def hasMultibody(",
        "def addRigidBodyFixedJoint(",
        "def getRigidBodyFixedJoint(",
        "def hasRigidBodyFixedJoint(",
        "def getRigidBodyFixedJoints(",
        "def getRigidBodyFixedJointCount(",
        "def get_parent_rigid_body(",
        "def get_child_rigid_body(",
        "def has_multibody_count(",
        "def get_rigid_body_count(",
    )
    for member in forbidden_stub_members:
        assert member not in stub


def _stub_class_block(stub: str, class_name: str) -> str:
    # Match the exact class header so a name like "RigidBody" does not also match
    # "RigidBodySolver". Stub headers look like "class Name:" or "class Name(Base):".
    for marker in (f"class {class_name}:", f"class {class_name}("):
        start = stub.find(marker)
        if start != -1:
            break
    else:
        raise AssertionError(f"class {class_name} not found in stub")
    end = stub.find("\nclass ", start + 1)
    if end == -1:
        end = len(stub)
    return stub[start:end]


def test_experimental_stub_places_rigid_surface_ccd_obstacle_on_rigid_body():
    sx = _simulation_experimental()
    repo_root = Path(__file__).resolve().parents[4]
    stub = (
        repo_root / "python" / "stubs" / "dartpy" / "simulation_experimental.pyi"
    ).read_text(encoding="utf-8")

    assert hasattr(sx.RigidBody, "is_deformable_surface_ccd_obstacle")
    assert not hasattr(sx.Link, "is_deformable_surface_ccd_obstacle")
    assert "is_deformable_surface_ccd_obstacle" in _stub_class_block(stub, "RigidBody")
    assert "is_deformable_surface_ccd_obstacle" not in _stub_class_block(stub, "Link")

    assert hasattr(sx.RigidBody, "is_deformable_ground_barrier")
    assert not hasattr(sx.Link, "is_deformable_ground_barrier")
    assert "is_deformable_ground_barrier" in _stub_class_block(stub, "RigidBody")
    assert "is_deformable_ground_barrier" not in _stub_class_block(stub, "Link")


def test_experimental_state_space_metadata_value_object():
    sx = _simulation_experimental()

    space = sx.StateSpace()
    assert space.dimension == 0
    assert space.num_variables == 0
    assert not space.is_finalized
    assert space.variable_names == []
    assert space.lower_bounds.tolist() == []
    assert space.upper_bounds.tolist() == []

    space.add_variable("arm.q", 2, lower=-1.0, upper=1.0).add_variables(
        ("arm.dq0", "arm.dq1"), lower=-10.0, upper=10.0
    )

    assert space.dimension == 4
    assert space.num_variables == 3
    assert space.variable_names == ["arm.q", "arm.dq0", "arm.dq1"]
    assert space.lower_bounds.tolist() == pytest.approx([-1.0, -1.0, -10.0, -10.0])
    assert space.upper_bounds.tolist() == pytest.approx([1.0, 1.0, 10.0, 10.0])
    assert space.has_variable("arm.q")
    assert not space.has_variable("missing")
    assert space.get_variable_index("arm.q") == 0
    assert space.get_variable_index("missing") is None

    variable = space.get_variable("arm.q")
    assert variable is not None
    assert variable.name == "arm.q"
    assert variable.start_index == 0
    assert variable.dimension == 2
    assert variable.lower_bound == pytest.approx(-1.0)
    assert variable.upper_bound == pytest.approx(1.0)
    assert space.get_variable("missing") is None

    variables = space.variables
    assert [entry.name for entry in variables] == ["arm.q", "arm.dq0", "arm.dq1"]
    assert [entry.start_index for entry in variables] == [0, 2, 3]

    space.finalize()
    assert space.is_finalized
    space.finalize()
    with pytest.raises(Exception, match="finalized"):
        space.add_variable("late", 1)

    with pytest.raises(Exception, match="already exists"):
        sx.StateSpace().add_variable("dup", 1).add_variable("dup", 1)
    with pytest.raises(Exception, match="dimension"):
        sx.StateSpace().add_variable("bad", 0)
    with pytest.raises(Exception, match="lower bound"):
        sx.StateSpace().add_variable("bad_bounds", 1, lower=2.0, upper=1.0)


def test_experimental_world_smoke():
    sx = _simulation_experimental()

    world = sx.World()
    assert not world.is_simulation_mode
    assert world.num_multibodies == 0
    assert world.num_loop_closures == 0
    assert world.num_rigid_bodies == 0

    multibody = world.add_multibody("robot")
    assert multibody.name == "robot"
    assert multibody.is_valid
    assert multibody.num_links == 0
    assert multibody.num_joints == 0
    assert multibody.num_dofs == 0

    multibody.name = "renamed_robot"
    assert multibody.name == "renamed_robot"
    assert world.has_multibody("renamed_robot")
    assert not world.has_multibody("missing")
    assert world.get_multibody("renamed_robot").name == "renamed_robot"
    assert world.get_multibody("missing") is None
    with pytest.raises(Exception, match="already exists"):
        world.add_multibody("renamed_robot")
    with pytest.raises(Exception, match="cannot be empty"):
        multibody.name = ""
    assert multibody.name == "renamed_robot"
    assert world.num_multibodies == 1
    assert world.num_loop_closures == 0

    auto_multibody_world = sx.World()
    auto_multibody_world.add_multibody("multibody_001")
    generated_multibody = auto_multibody_world.add_multibody("")
    assert generated_multibody.name == "multibody_002"
    assert auto_multibody_world.has_multibody("multibody_001")
    assert auto_multibody_world.has_multibody("multibody_002")
    assert auto_multibody_world.num_multibodies == 2

    rigid_body = world.add_rigid_body("box")
    assert rigid_body.name == "box"
    assert world.has_rigid_body("box")
    assert not world.has_rigid_body("missing")
    assert world.num_rigid_bodies == 1
    with pytest.raises(Exception, match="already exists"):
        world.add_rigid_body("box")
    assert world.num_rigid_bodies == 1

    auto_world = sx.World()
    auto_world.add_rigid_body("rigid_body_001")
    generated_body = auto_world.add_rigid_body("")
    assert generated_body.name == "rigid_body_002"
    assert auto_world.num_rigid_bodies == 2

    world.enter_simulation_mode()
    assert world.is_simulation_mode
    world.update_kinematics()

    world.clear()
    assert not world.is_simulation_mode
    assert not multibody.is_valid
    assert not rigid_body.is_valid
    assert world.num_multibodies == 0
    assert world.num_loop_closures == 0
    assert world.num_rigid_bodies == 0


def test_experimental_world_rigid_body_fixed_joint_projects_captured_pose():
    sx = _simulation_experimental()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    base = world.add_rigid_body("base")
    base.is_static = True
    link = world.add_rigid_body("link", position=(1.0, 0.0, 0.0))

    joint = world.add_rigid_body_fixed_joint("base_to_link", base, link)
    assert joint.name == "base_to_link"
    assert joint.type == sx.JointType.FIXED
    assert joint.num_dofs == 0
    assert joint.parent_rigid_body.name == "base"
    assert joint.child_rigid_body.name == "link"
    assert world.has_rigid_body_fixed_joint("base_to_link")
    assert not world.has_rigid_body_fixed_joint("missing")
    assert world.num_rigid_body_fixed_joints == 1
    assert world.get_rigid_body_fixed_joint("base_to_link").child_rigid_body == link
    assert world.get_rigid_body_fixed_joint("missing") is None
    fixed_joints = world.get_rigid_body_fixed_joints()
    assert len(fixed_joints) == 1
    assert fixed_joints[0].parent_rigid_body == base
    with pytest.raises(Exception, match="not a multibody Link"):
        _ = joint.parent_link
    with pytest.raises(Exception, match="not a multibody Link"):
        _ = joint.child_link
    with pytest.raises(Exception, match="already exists"):
        world.add_rigid_body_fixed_joint("base_to_link", base, link)

    drifted_pose = np.eye(4)
    drifted_pose[:3, 3] = (1.25, 0.0, 0.0)
    link.transform = drifted_pose

    world.enter_simulation_mode()
    world.step()

    assert float(link.translation[0]) == pytest.approx(1.0, abs=0.05)
    assert float(link.linear_velocity[0]) < 0.0
    with pytest.raises(Exception, match="simulation mode"):
        world.add_rigid_body_fixed_joint("late_joint", base, link)


def test_experimental_world_rigid_body_fixed_joint_list_keeps_world_alive():
    sx = _simulation_experimental()

    def build_joints_from_temporary_world():
        world = sx.World()
        base = world.add_rigid_body("base")
        link = world.add_rigid_body("link")
        world.add_rigid_body_fixed_joint("base_to_link", base, link)
        return world.get_rigid_body_fixed_joints()

    fixed_joints = build_joints_from_temporary_world()
    gc.collect()

    assert len(fixed_joints) == 1
    assert fixed_joints[0].name == "base_to_link"
    assert fixed_joints[0].parent_rigid_body.name == "base"
    assert fixed_joints[0].child_rigid_body.name == "link"


def test_experimental_multibody_link_joint_common_path():
    sx = _simulation_experimental()

    world = sx.World()
    arm = world.add_multibody("arm")

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
    tool = world.add_fixed_frame(
        "tool",
        forearm,
        offset=_translation_transform(1.0, 0.0, 0.0),
    )
    slider = arm.add_link(
        "slider",
        parent=base,
        joint=sx.JointSpec(
            name="rail",
            type=sx.JointType.PRISMATIC,
            axis=(1.0, 0.0, 0.0),
        ),
    )

    assert forearm.name == "forearm"
    assert arm.num_links == 3
    assert arm.num_joints == 2
    assert arm.num_dofs == 2
    assert [link.name for link in arm.links] == ["base", "forearm", "slider"]
    assert [joint.name for joint in arm.joints] == ["elbow", "rail"]
    assert arm.link_names == ["base", "forearm", "slider"]
    assert arm.joint_names == ["elbow", "rail"]
    with pytest.raises(Exception, match="already exists"):
        arm.add_link("base")
    assert arm.num_links == 3
    assert arm.num_joints == 2
    with pytest.raises(Exception, match="already exists"):
        arm.add_link(
            "forearm",
            parent=base,
            joint=sx.JointSpec(name="wrist", type=sx.JointType.REVOLUTE),
        )
    assert arm.num_links == 3
    assert arm.num_joints == 2
    with pytest.raises(Exception, match="already exists"):
        arm.add_link(
            "tool_link",
            parent=base,
            joint=sx.JointSpec(name="elbow", type=sx.JointType.REVOLUTE),
        )
    assert arm.num_links == 3
    assert arm.num_joints == 2
    other = world.add_multibody("other")
    other_base = other.add_link("other_base")
    with pytest.raises(Exception, match="does not belong"):
        arm.add_link(
            "foreign_child",
            parent=other_base,
            joint=sx.JointSpec(name="foreign_joint", type=sx.JointType.REVOLUTE),
        )
    assert arm.get_link("base").name == "base"
    assert arm.get_link("missing") is None

    joint = forearm.parent_joint
    assert joint.is_valid
    assert joint.name == "elbow"
    assert joint.type == sx.JointType.REVOLUTE
    assert joint.axis.tolist() == pytest.approx([0.0, 0.0, 1.0])
    assert joint.num_dofs == 1
    assert joint.position.tolist() == pytest.approx([0.0])
    assert joint.velocity.tolist() == pytest.approx([0.0])
    assert joint.parent_link.name == "base"
    assert joint.child_link.name == "forearm"
    with pytest.raises(Exception, match="not a rigid body"):
        _ = joint.parent_rigid_body
    with pytest.raises(Exception, match="not a rigid body"):
        _ = joint.child_rigid_body
    assert arm.get_joint("elbow").child_link.name == "forearm"
    assert arm.get_joint("missing") is None

    joint.position = [0.25]
    assert joint.position.tolist() == pytest.approx([0.25])
    joint.position = np.asarray([0.5], dtype=float)
    assert joint.position.tolist() == pytest.approx([0.5])
    joint.position = [math.pi / 2.0]

    joint.velocity = (-0.75,)
    assert joint.velocity.tolist() == pytest.approx([-0.75])
    joint.velocity = [1.25]
    assert joint.velocity.tolist() == pytest.approx([1.25])

    slider_joint = slider.parent_joint
    slider_joint.position = [2.0]

    # This test exercises construction and forward kinematics, so disable
    # gravity and joint velocities to keep the step from changing the pose.
    world.gravity = (0.0, 0.0, 0.0)
    joint.velocity = [0.0]
    slider_joint.velocity = [0.0]

    world.enter_simulation_mode()
    world.sync(sx.WorldSyncStage.KINEMATICS)
    assert tool.translation.tolist() == pytest.approx([0.0, 1.0, 0.0])
    base_to_tool = tool.relative_transform(base)
    assert base_to_tool[:3, 3].tolist() == pytest.approx([0.0, 1.0, 0.0])
    assert base_to_tool[:3, :3].reshape(9).tolist() == pytest.approx(
        [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        abs=1e-12,
    )
    assert slider.translation.tolist() == pytest.approx([2.0, 0.0, 0.0])
    assert world.time == pytest.approx(0.0)
    assert world.frame == 0

    world.step()

    assert world.is_simulation_mode
    assert joint.position.tolist() == pytest.approx([math.pi / 2.0])
    assert joint.velocity.tolist() == pytest.approx([0.0])
    assert tool.translation.tolist() == pytest.approx([0.0, 1.0, 0.0])


def test_link_local_transform_includes_joint_motion():
    sx = _simulation_experimental()

    world = sx.World()
    arm = world.add_multibody("arm")
    base = arm.add_link("base")
    link = arm.add_link(
        "link",
        parent=base,
        joint=sx.JointSpec(
            name="joint",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 0.0, 1.0),
        ),
    )

    world.enter_simulation_mode()
    link.parent_joint.position = [math.pi / 2.0]
    world.sync(sx.WorldSyncStage.KINEMATICS)

    local = link.local_transform
    relative_to_parent = link.relative_transform(link.parent_frame)

    # local_transform includes the joint motion, so it matches the
    # parent-relative transform rather than the bare mounting offset.
    assert local.reshape(16).tolist() == pytest.approx(
        relative_to_parent.reshape(16).tolist()
    )
    assert local[:3, :3].reshape(9).tolist() == pytest.approx(
        [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        abs=1e-12,
    )


def test_experimental_add_skeleton_imports_legacy_revolute_tree():
    sx = _simulation_experimental()

    skeleton = dart.Skeleton("py_loader")
    properties = dart.RevoluteJointProperties()
    properties.mAxis = np.asarray([0.0, 1.0, 0.0], dtype=float)
    joint, body = skeleton.create_revolute_joint_and_body_node_pair(
        None, properties
    )
    joint.set_name("hinge")
    joint.set_position(0, 0.2)
    joint.set_velocity(0, -0.3)

    options = sx.SkeletonLoadOptions()
    options.root_anchor_prefix = "py_anchor_"

    world = sx.World()
    multibody = sx.add_skeleton(world, skeleton, options)

    assert multibody.name == "py_loader"
    assert multibody.num_links == 2
    assert multibody.num_joints == 1
    assert multibody.num_dofs == 1
    assert multibody.get_link(f"py_anchor_{body.get_name()}") is not None

    loaded_link = multibody.get_link(body.get_name())
    assert loaded_link is not None
    loaded_joint = loaded_link.parent_joint
    assert loaded_joint.name == "hinge"
    assert loaded_joint.type == sx.JointType.REVOLUTE
    assert loaded_joint.axis.tolist() == pytest.approx([0.0, 1.0, 0.0])
    assert loaded_joint.position.tolist() == pytest.approx([0.2])
    assert loaded_joint.velocity.tolist() == pytest.approx([-0.3])


def test_experimental_add_skeleton_loads_uri():
    sx = _simulation_experimental()

    options = sx.SkeletonLoadOptions()
    options.root_anchor_prefix = "py_uri_anchor_"

    world = sx.World()
    multibody = sx.add_skeleton(
        world, "dart://sample/skel/test/single_pendulum.skel", options
    )

    assert multibody.name == "single_pendulum"
    assert multibody.num_links == 2
    assert multibody.num_joints == 1
    assert multibody.num_dofs == 1
    assert multibody.get_link("py_uri_anchor_link 1") is not None

    loaded_link = multibody.get_link("link 1")
    assert loaded_link is not None
    loaded_joint = loaded_link.parent_joint
    assert loaded_joint.name == "joint 1"
    assert loaded_joint.type == sx.JointType.REVOLUTE
    assert loaded_joint.axis.tolist() == pytest.approx([0.0, 0.0, 1.0])
    assert loaded_link.has_collision_shape
    assert loaded_link.collision_shape.type == sx.CollisionShapeType.BOX
    assert loaded_link.collision_shape.half_extents.tolist() == pytest.approx(
        [0.05, 0.1, 0.15]
    )


def test_experimental_add_skeleton_uri_accepts_read_options():
    sx = _simulation_experimental()

    read_options = sx.ReadOptions()
    assert read_options.format == sx.ModelFormat.AUTO
    assert read_options.sdf_default_root_joint_type == sx.RootJointType.FLOATING
    read_options.format = sx.ModelFormat.SKEL
    read_options.sdf_default_root_joint_type = sx.RootJointType.FIXED
    read_options.add_package_directory("unused", "/tmp")

    load_options = sx.SkeletonLoadOptions()
    load_options.root_anchor_prefix = "py_read_options_anchor_"

    world = sx.World()
    multibody = sx.add_skeleton(
        world,
        "dart://sample/skel/test/single_pendulum.skel",
        read_options,
        load_options,
    )

    assert multibody.name == "single_pendulum"
    assert multibody.get_link("py_read_options_anchor_link 1") is not None

    wrong_format = sx.ReadOptions()
    wrong_format.format = sx.ModelFormat.SDF
    with pytest.raises(Exception, match="Failed to read Skeleton"):
        sx.add_skeleton(
            sx.World(),
            "dart://sample/skel/test/single_pendulum.skel",
            wrong_format,
        )


def test_experimental_add_world_imports_legacy_world():
    sx = _simulation_experimental()

    legacy_world = dart.World("py_legacy_world")
    skeleton = dart.Skeleton("py_world_loader")
    properties = dart.RevoluteJointProperties()
    properties.mAxis = np.asarray([0.0, 1.0, 0.0], dtype=float)
    joint, body = skeleton.create_revolute_joint_and_body_node_pair(
        None, properties
    )
    joint.set_name("hinge")
    joint.set_position(0, 0.25)
    legacy_world.add_skeleton(skeleton)

    options = sx.SkeletonLoadOptions()
    options.root_anchor_prefix = "py_world_anchor_"

    world = sx.World()
    multibodies = sx.add_world(world, legacy_world, options)

    assert len(multibodies) == 1
    assert world.num_multibodies == 1
    assert multibodies[0].name == "py_world_loader"
    assert multibodies[0].get_link(f"py_world_anchor_{body.get_name()}") is not None

    loaded_link = multibodies[0].get_link(body.get_name())
    assert loaded_link is not None
    loaded_joint = loaded_link.parent_joint
    assert loaded_joint.name == "hinge"
    assert loaded_joint.position.tolist() == pytest.approx([0.25])


def test_experimental_add_world_rejects_unsupported_world_without_mutation():
    sx = _simulation_experimental()

    legacy_world = dart.World("py_unsupported_world")
    supported = dart.Skeleton("py_supported_world_loader")
    supported.create_revolute_joint_and_body_node_pair()
    legacy_world.add_skeleton(supported)

    unsupported = dart.Skeleton("py_unsupported_euler_world_loader")
    unsupported.create_euler_joint_and_body_node_pair()
    legacy_world.add_skeleton(unsupported)

    world = sx.World()
    with pytest.raises(Exception, match="Cannot translate legacy joint"):
        sx.add_world(world, legacy_world)
    assert world.num_multibodies == 0


def test_experimental_add_world_rejects_name_conflict_without_mutation():
    sx = _simulation_experimental()

    legacy_world = dart.World("py_conflict_world")
    first = dart.Skeleton("py_first_world_loader")
    first.create_revolute_joint_and_body_node_pair()
    legacy_world.add_skeleton(first)

    second = dart.Skeleton("py_second_world_loader")
    second.create_revolute_joint_and_body_node_pair()
    legacy_world.add_skeleton(second)

    world = sx.World()
    world.add_multibody("py_second_world_loader")

    with pytest.raises(Exception, match="already contains a Multibody"):
        sx.add_world(world, legacy_world)
    assert world.num_multibodies == 1
    assert world.get_multibody("py_first_world_loader") is None


def test_experimental_add_world_rejects_joint_axis_conflict_without_mutation():
    sx = _simulation_experimental()

    legacy_world = dart.World("py_axis_conflict_world")
    supported = dart.Skeleton("py_supported_world_loader")
    supported.create_revolute_joint_and_body_node_pair()
    legacy_world.add_skeleton(supported)

    invalid = dart.Skeleton("py_parallel_universal_axis")
    joint, _ = invalid.create_universal_joint_and_body_node_pair()
    joint.set_name("py_bad_universal")
    joint.set_axis1(np.asarray([0.0, 0.0, 1.0], dtype=float))
    joint.set_axis2(np.asarray([0.0, 0.0, 1.0], dtype=float))
    legacy_world.add_skeleton(invalid)

    world = sx.World()
    with pytest.raises(Exception, match="axis must not be parallel"):
        sx.add_world(world, legacy_world)
    assert world.num_multibodies == 0
    assert world.get_multibody("py_supported_world_loader") is None


def test_experimental_add_world_rejects_invalid_body_inertia_without_mutation():
    sx = _simulation_experimental()

    legacy_world = dart.World("py_invalid_inertia_world")
    supported = dart.Skeleton("py_supported_world_loader")
    supported.create_revolute_joint_and_body_node_pair()
    legacy_world.add_skeleton(supported)

    invalid = dart.Skeleton("py_zero_mass_world_loader")
    _, body = invalid.create_revolute_joint_and_body_node_pair()
    inertia = body.get_inertia()
    inertia.set_mass(0.0)
    body.set_inertia(inertia)
    legacy_world.add_skeleton(invalid)

    world = sx.World()
    with pytest.raises(Exception, match="mass must be positive"):
        sx.add_world(world, legacy_world)
    assert world.num_multibodies == 0
    assert world.get_multibody("py_supported_world_loader") is None


def test_experimental_add_world_loads_uri():
    sx = _simulation_experimental()

    options = sx.SkeletonLoadOptions()
    options.root_anchor_prefix = "py_world_uri_anchor_"

    world = sx.World()
    multibodies = sx.add_world(
        world, "dart://sample/skel/test/single_pendulum.skel", options
    )

    assert len(multibodies) == 1
    assert world.num_multibodies == 1
    assert multibodies[0].name == "single_pendulum"
    assert multibodies[0].get_link("py_world_uri_anchor_link 1") is not None


def test_experimental_add_world_uri_accepts_read_options():
    sx = _simulation_experimental()

    read_options = sx.ReadOptions()
    read_options.format = sx.ModelFormat.SKEL

    load_options = sx.SkeletonLoadOptions()
    load_options.root_anchor_prefix = "py_world_read_options_anchor_"

    world = sx.World()
    multibodies = sx.add_world(
        world,
        "dart://sample/skel/test/single_pendulum.skel",
        read_options,
        load_options,
    )

    assert len(multibodies) == 1
    assert multibodies[0].name == "single_pendulum"
    assert (
        multibodies[0].get_link("py_world_read_options_anchor_link 1")
        is not None
    )

    wrong_format = sx.ReadOptions()
    wrong_format.format = sx.ModelFormat.SDF
    with pytest.raises(Exception, match="Failed to read World"):
        sx.add_world(
            sx.World(),
            "dart://sample/skel/test/single_pendulum.skel",
            wrong_format,
        )


def test_experimental_add_world_results_keep_world_alive():
    sx = _simulation_experimental()

    options = sx.SkeletonLoadOptions()
    options.root_anchor_prefix = "py_world_lifetime_anchor_"

    multibodies = sx.add_world(
        sx.World(), "dart://sample/skel/test/single_pendulum.skel", options
    )
    gc.collect()

    assert multibodies[0].name == "single_pendulum"
    assert multibodies[0].get_link("py_world_lifetime_anchor_link 1") is not None


def test_experimental_loop_closure_topology_api():
    sx = _simulation_experimental()

    world = sx.World()
    arm = world.add_multibody("four_bar")
    base = arm.add_link("base")
    coupler = arm.add_link(
        "coupler",
        parent=base,
        joint=sx.JointSpec(name="shoulder", type=sx.JointType.REVOLUTE),
    )
    ground = world.add_rigid_body("ground")

    offset_a = _translation_transform(0.5, 0.0, 0.0)
    offset_b = _translation_transform(-0.5, 0.0, 0.0)
    spec = sx.LoopClosureSpec(
        frame_a=coupler,
        frame_b=ground,
        family=sx.LoopClosureFamily.RIGID,
        offset_a=offset_a,
        offset_b=offset_b,
    )
    assert spec.frame_a == coupler
    assert spec.frame_b == ground
    assert spec.family == sx.LoopClosureFamily.RIGID
    assert spec.offset_a.reshape(16).tolist() == pytest.approx(
        np.asarray(offset_a).reshape(16).tolist()
    )

    closure = world.add_loop_closure("closing_bar", spec)

    assert closure.is_valid
    assert closure.name == "closing_bar"
    assert closure.family == sx.LoopClosureFamily.RIGID
    assert closure.frame_a == coupler
    assert closure.frame_b == ground
    assert closure.offset_a.reshape(16).tolist() == pytest.approx(
        np.asarray(offset_a).reshape(16).tolist()
    )
    assert closure.offset_b.reshape(16).tolist() == pytest.approx(
        np.asarray(offset_b).reshape(16).tolist()
    )

    runtime_policy = closure.runtime_policy
    assert runtime_policy.enabled is True
    assert runtime_policy.kinematics == sx.ClosureKinematicsPolicy.RESIDUAL_ONLY
    assert runtime_policy.dynamics == sx.ClosureDynamicsPolicy.RESIDUAL_ONLY

    runtime_policy.enabled = False
    runtime_policy.kinematics = sx.ClosureKinematicsPolicy.PROJECT
    runtime_policy.dynamics = sx.ClosureDynamicsPolicy.SOLVE
    closure.runtime_policy = runtime_policy

    updated_policy = closure.runtime_policy
    assert updated_policy.enabled is False
    assert updated_policy.kinematics == sx.ClosureKinematicsPolicy.PROJECT
    assert updated_policy.dynamics == sx.ClosureDynamicsPolicy.SOLVE
    assert closure.enabled is False
    assert closure.kinematics == sx.ClosureKinematicsPolicy.PROJECT
    assert closure.dynamics == sx.ClosureDynamicsPolicy.SOLVE

    closure.enabled = True
    closure.kinematics = sx.ClosureKinematicsPolicy.RESIDUAL_ONLY
    closure.dynamics = sx.ClosureDynamicsPolicy.RESIDUAL_ONLY
    assert closure.runtime_policy.enabled is True
    assert closure.runtime_policy.kinematics == sx.ClosureKinematicsPolicy.RESIDUAL_ONLY
    assert closure.runtime_policy.dynamics == sx.ClosureDynamicsPolicy.RESIDUAL_ONLY

    closure.runtime_policy = sx.LoopClosureRuntimePolicy(
        enabled=True,
        kinematics=sx.ClosureKinematicsPolicy.RESIDUAL_ONLY,
        dynamics=sx.ClosureDynamicsPolicy.RESIDUAL_ONLY,
    )
    assert closure.runtime_policy.enabled is True

    assert world.num_loop_closures == 1
    assert world.has_loop_closure("closing_bar")
    assert world.get_loop_closure("closing_bar").frame_a == coupler
    assert world.get_loop_closure("missing") is None

    auto_closure = world.add_loop_closure(
        frame_a=base,
        frame_b=ground,
        family=sx.LoopClosureFamily.POINT,
    )
    assert auto_closure.name == "loop_closure_001"
    assert auto_closure.family == sx.LoopClosureFamily.POINT

    spec_auto_closure = world.add_loop_closure(
        sx.LoopClosureSpec(
            frame_a=base,
            frame_b=ground,
            family=sx.LoopClosureFamily.DISTANCE,
        )
    )
    assert spec_auto_closure.name == "loop_closure_002"
    assert spec_auto_closure.family == sx.LoopClosureFamily.DISTANCE
    assert world.num_loop_closures == 3

    world.enter_simulation_mode()
    residual = closure.compute_residual()
    assert isinstance(residual, sx.LoopClosureResidual)
    assert residual.enabled is True
    assert residual.active is True
    assert residual.coordinates == sx.LoopClosureResidualCoordinates.WORLD
    assert residual.force_available is False
    assert residual.value.tolist() == pytest.approx([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert residual.norm == pytest.approx(1.0)

    coupler.parent_joint.position = [math.pi / 2.0]
    world.sync(sx.WorldSyncStage.KINEMATICS)
    residual = closure.compute_residual()
    assert residual.value.tolist() == pytest.approx(
        [0.5, 0.5, 0.0, 0.0, 0.0, math.pi / 2.0]
    )


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
    world.sync()
    assert world.time == pytest.approx(0.0)
    assert world.frame == 0
    world.sync(sx.WorldSyncStage.KINEMATICS)
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
    assert box.linear_velocity.tolist() == pytest.approx([1.0, 0.0, 0.0])
    assert box.angular_velocity.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert box.mass == pytest.approx(2.0)
    assert box.inertia.reshape(9).tolist() == pytest.approx(
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    assert box.force.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert box.torque.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert world.has_rigid_body("box")
    assert world.get_rigid_body("box") == box
    assert world.get_rigid_body("missing") is None

    sensor = world.add_fixed_frame(
        "box_sensor",
        box,
        offset=_translation_transform(0.0, 1.0, 0.0),
    )
    box.transform = _translation_transform(4.0, 5.0, 6.0)
    assert box.translation.tolist() == pytest.approx([4.0, 5.0, 6.0])
    assert sensor.translation.tolist() == pytest.approx([4.0, 6.0, 6.0])

    box.linear_velocity = (2.0, 0.0, 0.0)
    box.angular_velocity = (0.0, 0.0, 0.5)
    assert box.linear_velocity.tolist() == pytest.approx([2.0, 0.0, 0.0])
    assert box.angular_velocity.tolist() == pytest.approx([0.0, 0.0, 0.5])
    box.angular_velocity = (0.0, 0.0, 0.0)

    box.mass = 5.0
    box.inertia = ((3.0, 0.0, 0.0), (0.0, 4.0, 0.0), (0.0, 0.0, 5.0))
    assert box.mass == pytest.approx(5.0)
    assert box.inertia.reshape(9).tolist() == pytest.approx(
        [3.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 5.0]
    )
    box.mass = 2.0

    box.force = (0.0, 2.0, 0.0)
    box.apply_force((0.0, 3.0, 0.0))
    assert box.force.tolist() == pytest.approx([0.0, 5.0, 0.0])
    box.clear_force()
    assert box.force.tolist() == pytest.approx([0.0, 0.0, 0.0])

    box.torque = (1.0, 0.0, 0.0)
    box.apply_torque((2.0, 0.0, 0.0))
    assert box.torque.tolist() == pytest.approx([3.0, 0.0, 0.0])
    box.clear_torque()
    assert box.torque.tolist() == pytest.approx([0.0, 0.0, 0.0])

    world.gravity = (0.0, 0.0, 0.0)
    world.step(n=0)
    assert not world.is_simulation_mode
    assert world.time == pytest.approx(0.0)
    assert world.frame == 0
    with pytest.raises(Exception, match="non-negative step count"):
        world.step(n=-1)

    world.step(n=3)

    assert world.is_simulation_mode
    assert world.time == pytest.approx(0.03)
    assert world.frame == 3
    assert box.translation.tolist() == pytest.approx([4.06, 5.0, 6.0])
    assert sensor.translation.tolist() == pytest.approx([4.06, 6.0, 6.0])


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
    world.gravity = (0.0, 0.0, 0.0)
    world.step()

    assert box.translation.tolist() == pytest.approx([2.1, 3.0, 4.0])


def test_experimental_world_gravity():
    sx = _simulation_experimental()

    world = sx.World(time_step=0.1)
    assert world.gravity.tolist() == pytest.approx([0.0, 0.0, -9.81])

    world.gravity = (0.0, -2.0, 0.0)
    assert world.gravity.tolist() == pytest.approx([0.0, -2.0, 0.0])

    # Gravity accelerates a free body during stepping, independent of mass.
    world.gravity = (0.0, 0.0, -10.0)
    box = world.add_rigid_body("box", mass=3.0, position=(0.0, 0.0, 5.0))
    world.step()

    dt = 0.1
    gravity_z = -10.0
    # Semi-implicit Euler: v = g dt, x = x0 + v dt = x0 + g dt^2.
    assert box.linear_velocity.tolist() == pytest.approx([0.0, 0.0, gravity_z * dt])
    assert box.translation.tolist() == pytest.approx(
        [0.0, 0.0, 5.0 + gravity_z * dt * dt]
    )

    with pytest.raises(Exception, match="finite"):
        world.gravity = (float("nan"), 0.0, 0.0)


def test_experimental_multibody_options_selector():
    sx = _simulation_experimental()

    world = sx.World()
    # Solver config is a value object; selection is by documented method-family
    # name only (no per-setting setters).
    assert world.multibody_options.integration_family == "semi-implicit"
    world.multibody_options = sx.MultibodyOptions(
        integration_family="variational integrator"
    )
    assert world.multibody_options.integration_family == "variational integrator"
    with pytest.raises(Exception):
        world.multibody_options = sx.MultibodyOptions(integration_family="nonsense")
    # A rejected assignment leaves the previous valid selection in place.
    assert world.multibody_options.integration_family == "variational integrator"

    # The variational integrator runs on the default step() path.
    arm = world.add_multibody("pendulum")
    base = arm.add_link("base")
    bob = arm.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge", type=sx.JointType.REVOLUTE, axis=(0.0, 1.0, 0.0)
        ),
    )
    bob.mass = 1.0
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))
    world.time_step = 1e-3
    world.enter_simulation_mode()
    world.step(10)
    assert world.time == pytest.approx(10e-3)


def test_experimental_rigid_body_dynamic_quantities():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    box = world.add_rigid_body(
        "box",
        mass=2.0,
        position=(0.0, 0.0, 5.0),
        linear_velocity=(3.0, 0.0, 0.0),
        angular_velocity=(1.0, 0.0, 0.0),
    )
    box.inertia = ((2.0, 0.0, 0.0), (0.0, 4.0, 0.0), (0.0, 0.0, 8.0))

    assert box.linear_momentum.tolist() == pytest.approx([6.0, 0.0, 0.0])
    assert box.angular_momentum.tolist() == pytest.approx([2.0, 0.0, 0.0])
    assert box.kinetic_energy == pytest.approx(10.0)
    assert box.potential_energy == pytest.approx(98.1)


def test_experimental_multibody_forward_dynamics():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    # Single revolute pendulum, horizontal at q = 0 (center of mass offset L).
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    spec = sx.JointSpec(
        name="hinge",
        type=sx.JointType.REVOLUTE,
        axis=(0.0, 1.0, 0.0),
        transform_from_parent=offset,
    )
    assert spec.transform_from_parent[0, 3] == pytest.approx(length)
    bob = robot.add_link("bob", parent=base, joint=spec)

    mass = 2.0
    inertia_yy = 0.2
    bob.mass = mass
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.3))
    assert bob.mass == pytest.approx(2.0)

    # Prismatic joint aligned with gravity (built in design mode before stepping).
    slider = world.add_multibody("slider")
    rail_base = slider.add_link("base")
    carriage = slider.add_link(
        "carriage",
        parent=rail_base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    carriage.mass = 3.0

    hinge = bob.parent_joint
    hinge.force = [0.0]
    assert hinge.force.tolist() == pytest.approx([0.0])

    world.time_step = 0.001
    world.step()

    expected = 9.81 * mass * length / (inertia_yy + mass * length * length)
    assert hinge.acceleration.tolist()[0] == pytest.approx(expected)
    assert carriage.parent_joint.acceleration.tolist()[0] == pytest.approx(-9.81)


def test_experimental_screw_joint_dynamics():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("screw")
    base = robot.add_link("base")
    nut = robot.add_link(
        "nut",
        parent=base,
        joint=sx.JointSpec(name="helix", type=sx.JointType.SCREW, axis=(0.0, 0.0, 1.0)),
    )
    mass = 2.0
    inertia_zz = 0.1
    nut.mass = mass
    nut.inertia = ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, inertia_zz))

    joint = nut.parent_joint
    pitch = 0.5
    joint.pitch = pitch
    assert joint.pitch == pytest.approx(pitch)

    world.time_step = 0.001
    world.enter_simulation_mode()

    # M = I_zz + m pitch^2 about the screw axis.
    expected_mass = inertia_zz + mass * pitch * pitch
    assert robot.mass_matrix[0, 0] == pytest.approx(expected_mass)

    world.step()
    # Gravity drives the screw down: qddot = -m g pitch / M.
    expected_accel = -mass * 9.81 * pitch / expected_mass
    assert joint.acceleration.tolist()[0] == pytest.approx(expected_accel, abs=1e-9)


def test_experimental_universal_joint_dynamics():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("ujoint")
    base = robot.add_link("base")
    length = 0.9
    offset = np.eye(4)
    offset[0, 3] = length
    spec = sx.JointSpec(
        name="u",
        type=sx.JointType.UNIVERSAL,
        axis=(0.0, 0.0, 1.0),
        axis2=(0.0, 1.0, 0.0),
        transform_from_parent=offset,
    )
    distal = robot.add_link("distal", parent=base, joint=spec)
    mass = 2.0
    inertia_yy = 0.12
    inertia_zz = 0.2
    distal.mass = mass
    distal.inertia = (
        (0.05, 0.0, 0.0),
        (0.0, inertia_yy, 0.0),
        (0.0, 0.0, inertia_zz),
    )

    joint = distal.parent_joint
    assert joint.type == sx.JointType.UNIVERSAL
    assert joint.num_dofs == 2
    assert joint.axis2.tolist() == pytest.approx([0.0, 1.0, 0.0])

    world.enter_simulation_mode()

    # M and gravity at q = 0: axis (Z) and axis2 (Y) intersect at the origin and
    # the distal center of mass sits at (L, 0, 0).
    mass_matrix = robot.mass_matrix
    assert mass_matrix.shape == (2, 2)
    assert mass_matrix[0, 0] == pytest.approx(inertia_zz + mass * length**2)
    assert mass_matrix[1, 1] == pytest.approx(inertia_yy + mass * length**2)
    assert mass_matrix[0, 1] == pytest.approx(0.0, abs=1e-12)

    gravity = robot.gravity_forces
    assert gravity[0] == pytest.approx(0.0, abs=1e-12)
    assert gravity[1] == pytest.approx(-mass * 9.81 * length)

    # The Coriolis force must match the Christoffel-symbol expression derived
    # from the configuration-dependent mass matrix by finite differences. That
    # reference does not use the velocity-product term cJ, so agreement (at a
    # configuration where both joint velocities are nonzero) validates cJ.
    q = np.array([0.3, 0.5])
    qdot = np.array([0.7, 1.1])

    def mass_at(position):
        joint.position = position.tolist()
        return np.array(robot.mass_matrix)

    h = 1e-5
    dM = []
    for i in range(2):
        plus = q.copy()
        minus = q.copy()
        plus[i] += h
        minus[i] -= h
        dM.append((mass_at(plus) - mass_at(minus)) / (2.0 * h))

    expected = np.zeros(2)
    for k in range(2):
        for i in range(2):
            for j in range(2):
                christoffel = 0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j])
                expected[k] += christoffel * qdot[i] * qdot[j]

    joint.position = q.tolist()
    joint.velocity = qdot.tolist()
    coriolis = robot.coriolis_forces
    assert coriolis.tolist() == pytest.approx(expected.tolist(), abs=1e-6)
    assert np.linalg.norm(expected) > 1e-3


def test_experimental_planar_joint_dynamics():
    sx = _simulation_experimental()

    # Mass matrix and gravity at q = 0 (plane normal Y, in-plane axes X and -Z,
    # center of mass at the joint origin).
    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("planar")
    base = robot.add_link("base")
    slider = robot.add_link(
        "slider",
        parent=base,
        joint=sx.JointSpec(
            name="plane",
            type=sx.JointType.PLANAR,
            axis=(0.0, 1.0, 0.0),
            axis2=(1.0, 0.0, 0.0),
        ),
    )
    mass = 3.0
    inertia_yy = 0.15
    slider.mass = mass
    slider.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.2))

    joint = slider.parent_joint
    assert joint.type == sx.JointType.PLANAR
    assert joint.num_dofs == 3

    world.enter_simulation_mode()

    mass_matrix = robot.mass_matrix
    assert mass_matrix.shape == (3, 3)
    assert np.allclose(mass_matrix, np.diag([mass, mass, inertia_yy]), atol=1e-12)

    gravity = robot.gravity_forces
    assert gravity[0] == pytest.approx(0.0, abs=1e-12)
    assert gravity[1] == pytest.approx(-mass * 9.81)
    assert gravity[2] == pytest.approx(0.0, abs=1e-12)

    # Coriolis must match the Christoffel symbols of M(q) (finite differences).
    # A nonzero link offset couples rotation to the translations, so the
    # reference is nonzero and validates the velocity-product term cJ.
    offset_world = sx.World()
    offset_robot = offset_world.add_multibody("planar")
    offset_base = offset_robot.add_link("base")
    offset_mat = np.eye(4)
    offset_mat[0, 3] = 0.6
    offset_slider = offset_robot.add_link(
        "slider",
        parent=offset_base,
        joint=sx.JointSpec(
            name="plane",
            type=sx.JointType.PLANAR,
            axis=(0.0, 1.0, 0.0),
            axis2=(1.0, 0.0, 0.0),
            transform_from_parent=offset_mat,
        ),
    )
    offset_slider.mass = mass
    offset_slider.inertia = (
        (0.1, 0.0, 0.0),
        (0.0, 0.15, 0.0),
        (0.0, 0.0, 0.2),
    )
    offset_joint = offset_slider.parent_joint
    offset_world.enter_simulation_mode()

    q = np.array([0.2, -0.15, 0.5])
    qdot = np.array([0.7, -0.4, 1.1])

    def mass_at(position):
        offset_joint.position = position.tolist()
        return np.array(offset_robot.mass_matrix)

    h = 1e-5
    dM = []
    for i in range(3):
        plus = q.copy()
        minus = q.copy()
        plus[i] += h
        minus[i] -= h
        dM.append((mass_at(plus) - mass_at(minus)) / (2.0 * h))

    expected = np.zeros(3)
    for k in range(3):
        for i in range(3):
            for j in range(3):
                christoffel = 0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j])
                expected[k] += christoffel * qdot[i] * qdot[j]

    offset_joint.position = q.tolist()
    offset_joint.velocity = qdot.tolist()
    coriolis = offset_robot.coriolis_forces
    assert coriolis.tolist() == pytest.approx(expected.tolist(), abs=1e-6)
    assert np.linalg.norm(expected) > 1e-3


def test_experimental_ball_joint_dynamics():
    sx = _simulation_experimental()

    # Mass matrix and gravity at the identity orientation (center of mass offset
    # along X): M is the inertia about the ball center, gravity torque about Y.
    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("ball")
    base = robot.add_link("base")
    offset = np.eye(4)
    offset[0, 3] = 0.7
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="socket", type=sx.JointType.SPHERICAL, transform_from_parent=offset
        ),
    )
    mass = 2.0
    bob.mass = mass
    bob.inertia = ((0.05, 0.0, 0.0), (0.0, 0.12, 0.0), (0.0, 0.0, 0.2))

    joint = bob.parent_joint
    assert joint.type == sx.JointType.SPHERICAL
    assert joint.num_dofs == 3

    world.enter_simulation_mode()

    expected_mass = np.diag([0.05, 0.12 + mass * 0.7**2, 0.2 + mass * 0.7**2])
    assert np.allclose(robot.mass_matrix, expected_mass, atol=1e-12)
    gravity = robot.gravity_forces
    assert gravity[0] == pytest.approx(0.0, abs=1e-12)
    assert gravity[1] == pytest.approx(-mass * 9.81 * 0.7)
    assert gravity[2] == pytest.approx(0.0, abs=1e-12)

    # SO(3) manifold integration: torque-free isotropic spin keeps a constant
    # angular velocity and accumulates the rotation vector linearly.
    spin_world = sx.World()
    spin_world.gravity = (0.0, 0.0, 0.0)
    spin_robot = spin_world.add_multibody("ball")
    spin_base = spin_robot.add_link("base")
    spin_bob = spin_robot.add_link(
        "bob",
        parent=spin_base,
        joint=sx.JointSpec(name="socket", type=sx.JointType.SPHERICAL),
    )
    spin_bob.mass = 1.5
    spin_bob.inertia = ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))
    spin_joint = spin_bob.parent_joint
    omega = np.array([0.3, -0.5, 0.7])
    spin_joint.velocity = omega.tolist()

    dt = 0.001
    steps = 100
    spin_world.time_step = dt
    spin_world.enter_simulation_mode()
    for _ in range(steps):
        spin_world.step()

    assert spin_joint.velocity.tolist() == pytest.approx(omega.tolist(), abs=1e-9)
    assert spin_joint.position.tolist() == pytest.approx(
        (omega * dt * steps).tolist(), abs=1e-9
    )


def test_experimental_free_joint_dynamics():
    sx = _simulation_experimental()

    # Free fall under gravity: linear acceleration equals gravity, no rotation.
    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("floating")
    base = robot.add_link("base")
    body = robot.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 2.0
    body.inertia = ((0.05, 0.0, 0.0), (0.0, 0.12, 0.0), (0.0, 0.0, 0.2))

    joint = body.parent_joint
    assert joint.type == sx.JointType.FLOATING
    assert joint.num_dofs == 6

    dt = 0.01
    world.time_step = dt
    world.enter_simulation_mode()
    world.step()

    # Velocity ordering is [linear; angular].
    acceleration = joint.acceleration
    assert acceleration[:3].tolist() == pytest.approx([0.0, 0.0, -9.81], abs=1e-9)
    assert acceleration[3:].tolist() == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)
    position = joint.position
    assert position[:3].tolist() == pytest.approx(
        [0.0, 0.0, -9.81 * dt * dt], abs=1e-12
    )
    assert position[3:].tolist() == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)

    # Combined SE(3) integration: translation and spin both along Z stay
    # constant and accumulate linearly.
    spin_world = sx.World()
    spin_world.gravity = (0.0, 0.0, 0.0)
    spin_robot = spin_world.add_multibody("floating")
    spin_base = spin_robot.add_link("base")
    spin_body = spin_robot.add_link(
        "body",
        parent=spin_base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    spin_body.mass = 2.0
    spin_body.inertia = ((0.05, 0.0, 0.0), (0.0, 0.12, 0.0), (0.0, 0.0, 0.2))
    spin_joint = spin_body.parent_joint
    twist = [0.0, 0.0, 3.0, 0.0, 0.0, 2.0]
    spin_joint.velocity = twist

    steps = 100
    spin_world.time_step = dt
    spin_world.enter_simulation_mode()
    for _ in range(steps):
        spin_world.step()

    total = dt * steps
    assert spin_joint.velocity.tolist() == pytest.approx(twist, abs=1e-9)
    position = spin_joint.position
    assert position[:3].tolist() == pytest.approx([0.0, 0.0, 3.0 * total], abs=1e-9)
    assert position[3:].tolist() == pytest.approx([0.0, 0.0, 2.0 * total], abs=1e-9)


def test_experimental_link_center_of_mass_offset():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    # Link frame at the hinge; the mass is offset along X via the center of mass.
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge", type=sx.JointType.REVOLUTE, axis=(0.0, 1.0, 0.0)
        ),
    )
    mass = 2.0
    length = 1.5
    inertia_yy = 0.2
    bob.mass = mass
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.3))
    bob.center_of_mass = (length, 0.0, 0.0)
    assert bob.center_of_mass.tolist() == pytest.approx([length, 0.0, 0.0])

    world.enter_simulation_mode()

    # Parallel-axis mass matrix and horizontal-pendulum gravity torque.
    assert robot.mass_matrix[0, 0] == pytest.approx(inertia_yy + mass * length**2)
    assert robot.gravity_forces.tolist() == pytest.approx([-mass * 9.81 * length])


def test_experimental_multibody_dynamics_terms():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    # Single revolute pendulum, horizontal at q = 0 (center of mass offset L).
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    spec = sx.JointSpec(
        name="hinge",
        type=sx.JointType.REVOLUTE,
        axis=(0.0, 1.0, 0.0),
        transform_from_parent=offset,
    )
    bob = robot.add_link("bob", parent=base, joint=spec)
    mass = 2.0
    inertia_yy = 0.2
    bob.mass = mass
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.3))

    # A single revolute DOF has no Coriolis term even at nonzero velocity.
    hinge = bob.parent_joint
    hinge.velocity = [2.0]

    world.enter_simulation_mode()

    expected_mass = inertia_yy + mass * length * length
    mass_matrix = robot.mass_matrix
    assert mass_matrix.shape == (1, 1)
    assert mass_matrix[0, 0] == pytest.approx(expected_mass)

    inverse_mass = robot.inverse_mass_matrix
    assert inverse_mass.shape == (1, 1)
    assert inverse_mass[0, 0] == pytest.approx(1.0 / expected_mass)

    # Horizontal pendulum: gravity generalized force is -m g L.
    gravity_forces = robot.gravity_forces
    assert gravity_forces.tolist() == pytest.approx([-mass * 9.81 * length])

    assert robot.coriolis_forces.tolist() == pytest.approx([0.0])
    assert robot.coriolis_and_gravity_forces.tolist() == pytest.approx(
        gravity_forces.tolist()
    )


def test_experimental_multibody_equation_of_motion_consistency():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    robot = world.add_multibody("double_pendulum")
    base = robot.add_link("base")

    offset1 = np.eye(4)
    offset1[0, 3] = 0.7
    offset2 = np.eye(4)
    offset2[0, 3] = 0.6

    link1 = robot.add_link(
        "link1",
        parent=base,
        joint=sx.JointSpec(
            name="j1",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset1,
        ),
    )
    link1.mass = 1.5
    link1.inertia = ((0.05, 0.0, 0.0), (0.0, 0.08, 0.0), (0.0, 0.0, 0.05))

    link2 = robot.add_link(
        "link2",
        parent=link1,
        joint=sx.JointSpec(
            name="j2",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset2,
        ),
    )
    link2.mass = 1.0
    link2.inertia = ((0.04, 0.0, 0.0), (0.0, 0.06, 0.0), (0.0, 0.0, 0.04))

    joint1 = link1.parent_joint
    joint2 = link2.parent_joint
    joint1.position = [0.3]
    joint1.velocity = [1.1]
    joint1.force = [2.0]
    joint2.position = [-0.5]
    joint2.velocity = [0.7]
    joint2.force = [-1.5]

    world.time_step = 1e-3
    world.enter_simulation_mode()

    # Read the dynamics terms at the current (pre-step) state.
    mass_matrix = robot.mass_matrix
    coriolis = robot.coriolis_forces
    gravity_forces = robot.gravity_forces
    assert mass_matrix.shape == (2, 2)
    assert np.allclose(mass_matrix, mass_matrix.T)

    # step() solves the same state, so the equation of motion must hold.
    world.step()
    qddot = np.array([joint1.acceleration[0], joint2.acceleration[0]])
    tau = np.array([2.0, -1.5])
    residual = mass_matrix @ qddot + coriolis + gravity_forces - tau
    assert np.allclose(residual, np.zeros(2), atol=1e-9)


def test_experimental_multibody_inverse_dynamics():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset,
        ),
    )
    mass = 2.0
    inertia_yy = 0.2
    bob.mass = mass
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.3))

    world.enter_simulation_mode()

    # At q = 0, qdot = 0: tau = (I + m L^2) qddot + g, with g = -m g L.
    accel = 3.0
    tau = robot.compute_inverse_dynamics([accel])
    expected = (inertia_yy + mass * length * length) * accel - mass * 9.81 * length
    assert tau.tolist() == pytest.approx([expected])

    with pytest.raises(Exception, match="must match"):
        robot.compute_inverse_dynamics([0.0, 0.0])


def test_experimental_multibody_impulse_response():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset,
        ),
    )
    mass = 2.0
    inertia_yy = 0.2
    bob.mass = mass
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.3))

    world.enter_simulation_mode()

    impulse = 5.0
    delta_velocity = robot.compute_impulse_response([impulse])
    inertia_pivot = inertia_yy + mass * length * length
    assert delta_velocity.tolist() == pytest.approx([impulse / inertia_pivot])

    # Consistency: M dqdot = f.
    assert (robot.mass_matrix @ delta_velocity).tolist() == pytest.approx([impulse])

    with pytest.raises(Exception, match="must match"):
        robot.compute_impulse_response([0.0, 0.0])


def test_experimental_multibody_link_jacobian():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset,
        ),
    )
    bob.mass = 1.0

    world.enter_simulation_mode()

    jacobian = robot.get_jacobian(bob)
    assert jacobian.shape == (6, 1)
    # Body twist [axis; axis x p] = [0, 1, 0, 0, 0, -L].
    assert jacobian[:, 0].tolist() == pytest.approx([0.0, 1.0, 0.0, 0.0, 0.0, -length])

    # The fixed base cannot move: its Jacobian is zero.
    base_jacobian = robot.get_jacobian(base)
    assert base_jacobian.shape == (6, 1)
    assert np.allclose(base_jacobian, 0.0)


def test_experimental_multibody_link_world_jacobian():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset,
        ),
    )
    bob.mass = 1.0

    world.enter_simulation_mode()

    # At q = 0 the link frame is axis-aligned with world, so the world Jacobian
    # equals the body Jacobian: [0, 1, 0, 0, 0, -L].
    world_jacobian = robot.get_world_jacobian(bob)
    assert world_jacobian.shape == (6, 1)
    assert world_jacobian[:, 0].tolist() == pytest.approx(
        [0.0, 1.0, 0.0, 0.0, 0.0, -length]
    )


def test_experimental_multibody_dynamics_terms_no_dof():
    sx = _simulation_experimental()

    world = sx.World()
    robot = world.add_multibody("static_chain")
    base = robot.add_link("base")
    robot.add_link(
        "welded",
        parent=base,
        joint=sx.JointSpec(name="weld", type=sx.JointType.FIXED),
    )

    world.enter_simulation_mode()

    assert robot.num_dofs == 0
    assert robot.mass_matrix.size == 0
    assert robot.inverse_mass_matrix.size == 0
    assert robot.coriolis_forces.size == 0
    assert robot.gravity_forces.size == 0
    assert robot.coriolis_and_gravity_forces.size == 0


def test_experimental_joint_armature():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    length = 1.5
    offset = np.eye(4)
    offset[0, 3] = length
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset,
        ),
    )
    mass = 2.0
    inertia_yy = 0.2
    bob.mass = mass
    bob.inertia = ((0.1, 0.0, 0.0), (0.0, inertia_yy, 0.0), (0.0, 0.0, 0.3))

    joint = bob.parent_joint
    assert joint.armature.tolist() == pytest.approx([0.0])
    armature = 1.0
    joint.armature = [armature]
    assert joint.armature.tolist() == pytest.approx([armature])

    with pytest.raises(Exception, match="non-negative"):
        joint.armature = [-1.0]

    world.time_step = 0.001
    world.enter_simulation_mode()

    # Armature adds to the mass-matrix diagonal.
    expected_mass = inertia_yy + mass * length * length + armature
    assert robot.mass_matrix[0, 0] == pytest.approx(expected_mass)

    world.step()
    expected_accel = 9.81 * mass * length / expected_mass
    assert joint.acceleration.tolist()[0] == pytest.approx(expected_accel, abs=1e-9)


def test_experimental_joint_coulomb_friction():
    sx = _simulation_experimental()

    # Stiction: a driving force within the bound does not move the joint.
    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("slider")
    base = robot.add_link("base")
    carriage = robot.add_link(
        "carriage",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    carriage.mass = 2.0

    joint = carriage.parent_joint
    assert joint.coulomb_friction.tolist() == pytest.approx([0.0])
    joint.coulomb_friction = [10.0]
    joint.force = [5.0]

    with pytest.raises(Exception, match="non-negative"):
        joint.coulomb_friction = [-1.0]

    world.time_step = 0.01
    world.step(n=100)
    assert joint.velocity.tolist()[0] == pytest.approx(0.0, abs=1e-12)
    assert joint.position.tolist()[0] == pytest.approx(0.0, abs=1e-12)

    # Kinetic: exceeding the bound yields net velocity step (F - mu) / m * dt.
    world2 = sx.World()
    world2.gravity = (0.0, 0.0, 0.0)
    robot2 = world2.add_multibody("slider")
    base2 = robot2.add_link("base")
    carriage2 = robot2.add_link(
        "carriage",
        parent=base2,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    mass = 2.0
    carriage2.mass = mass
    joint2 = carriage2.parent_joint
    joint2.coulomb_friction = [10.0]
    joint2.force = [20.0]
    dt = 0.01
    world2.time_step = dt
    world2.step()
    expected_velocity = (20.0 - 10.0) / mass * dt
    assert joint2.velocity.tolist()[0] == pytest.approx(expected_velocity)


def test_experimental_joint_actuator_types():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("slider")
    base = robot.add_link("base")
    carriage = robot.add_link(
        "carriage",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    mass = 2.0
    carriage.mass = mass

    joint = carriage.parent_joint
    assert joint.actuator_type == sx.ActuatorType.FORCE
    joint.actuator_type = sx.ActuatorType.PASSIVE
    assert joint.actuator_type == sx.ActuatorType.PASSIVE

    # Passive ignores the commanded effort but still applies the spring.
    joint.force = [100.0]
    stiffness = 10.0
    joint.spring_stiffness = [stiffness]
    joint.rest_position = [0.0]
    joint.position = [1.0]

    world.time_step = 0.001
    world.step()
    assert joint.acceleration.tolist()[0] == pytest.approx(
        -stiffness * 1.0 / mass, abs=1e-9
    )

    # Unimplemented actuator types are rejected by the dynamics.
    joint.actuator_type = sx.ActuatorType.SERVO
    with pytest.raises(Exception, match="not yet implemented"):
        world.step()


def test_experimental_joint_velocity_actuator():
    sx = _simulation_experimental()

    # Single slider reaches its commanded velocity regardless of applied force.
    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("slider")
    base = robot.add_link("base")
    carriage = robot.add_link(
        "carriage",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    carriage.mass = 2.0

    joint = carriage.parent_joint
    assert joint.command_velocity.tolist() == pytest.approx([0.0])
    joint.actuator_type = sx.ActuatorType.VELOCITY
    joint.command_velocity = [0.5]
    joint.force = [100.0]  # ignored by the Velocity actuator

    world.time_step = 0.01
    world.step()
    assert joint.velocity.tolist()[0] == pytest.approx(0.5)

    # Coupled 2-link chain: both joints reach their (different) targets exactly.
    world2 = sx.World()
    world2.gravity = (0.0, 0.0, 0.0)
    robot2 = world2.add_multibody("double_pendulum")
    base2 = robot2.add_link("base")
    offset1 = np.eye(4)
    offset1[0, 3] = 0.7
    offset2 = np.eye(4)
    offset2[0, 3] = 0.6
    link1 = robot2.add_link(
        "link1",
        parent=base2,
        joint=sx.JointSpec(
            name="j1",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset1,
        ),
    )
    link1.mass = 1.5
    link1.inertia = ((0.05, 0.0, 0.0), (0.0, 0.08, 0.0), (0.0, 0.0, 0.05))
    link2 = robot2.add_link(
        "link2",
        parent=link1,
        joint=sx.JointSpec(
            name="j2",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset2,
        ),
    )
    link2.mass = 1.0
    link2.inertia = ((0.04, 0.0, 0.0), (0.0, 0.06, 0.0), (0.0, 0.0, 0.04))

    j1 = link1.parent_joint
    j2 = link2.parent_joint
    j1.actuator_type = sx.ActuatorType.VELOCITY
    j2.actuator_type = sx.ActuatorType.VELOCITY
    j1.command_velocity = [0.3]
    j2.command_velocity = [-0.4]

    world2.time_step = 0.005
    world2.step()
    assert j1.velocity.tolist()[0] == pytest.approx(0.3, abs=1e-9)
    assert j2.velocity.tolist()[0] == pytest.approx(-0.4, abs=1e-9)


def test_experimental_joint_spring_and_damping():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("slider")
    base = robot.add_link("base")
    carriage = robot.add_link(
        "carriage",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    carriage.mass = 2.0

    joint = carriage.parent_joint
    joint.spring_stiffness = [10.0]
    joint.damping_coefficient = [3.0]
    joint.rest_position = [0.0]
    assert joint.spring_stiffness.tolist() == pytest.approx([10.0])
    assert joint.damping_coefficient.tolist() == pytest.approx([3.0])

    joint.position = [0.5]
    joint.velocity = [2.0]

    world.time_step = 0.001
    world.step()

    expected = (-10.0 * 0.5 - 3.0 * 2.0) / 2.0
    assert joint.acceleration.tolist()[0] == pytest.approx(expected)


def test_experimental_joint_position_limit():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)
    robot = world.add_multibody("pendulum")
    base = robot.add_link("base")
    offset = np.eye(4)
    offset[0, 3] = 1.0
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=offset,
        ),
    )
    bob.mass = 1.0
    bob.inertia = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))

    joint = bob.parent_joint
    assert math.isinf(joint.position_upper_limits.tolist()[0])
    joint.set_position_limits([-math.inf], [0.5])
    assert joint.position_upper_limits.tolist()[0] == pytest.approx(0.5)

    world.time_step = 0.005
    world.step(n=400)

    # Gravity drives the joint toward +pi/2 but it stops at the upper limit.
    assert joint.position.tolist()[0] == pytest.approx(0.5, abs=1e-9)
    assert joint.velocity.tolist()[0] == pytest.approx(0.0, abs=1e-9)


def test_experimental_joint_effort_limit():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("slider")
    base = robot.add_link("base")
    carriage = robot.add_link(
        "carriage",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    mass = 2.0
    carriage.mass = mass

    joint = carriage.parent_joint
    assert math.isinf(joint.effort_upper_limits.tolist()[0])
    joint.set_effort_limits([-10.0], [10.0])
    joint.force = [100.0]  # far above the limit

    with pytest.raises(Exception, match="must not exceed upper limits"):
        joint.set_effort_limits([1.0], [0.0])

    world.time_step = 0.01
    world.step()

    # The applied effort is clamped to the limit, so qddot = 10 / mass.
    assert joint.acceleration.tolist()[0] == pytest.approx(10.0 / mass)


def test_experimental_joint_velocity_limit():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("slider")
    base = robot.add_link("base")
    carriage = robot.add_link(
        "carriage",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    carriage.mass = 2.0

    joint = carriage.parent_joint
    assert math.isinf(joint.velocity_upper_limits.tolist()[0])
    velocity_limit = 0.1
    joint.set_velocity_limits([-velocity_limit], [velocity_limit])
    joint.force = [10.0]  # accelerates the slider

    world.time_step = 0.01
    for _ in range(200):
        world.step()
        assert joint.velocity.tolist()[0] <= velocity_limit + 1e-12

    # Under continued forcing the velocity saturates exactly at the limit.
    assert joint.velocity.tolist()[0] == pytest.approx(velocity_limit)


def test_experimental_collision_query():
    sx = _simulation_experimental()

    world = sx.World()

    body_a = world.add_rigid_body("a", position=(0.0, 0.0, 0.0))
    body_a.set_collision_shape(sx.CollisionShape.sphere(1.0))
    body_b = world.add_rigid_body("b", position=(1.2, 0.0, 0.0))
    body_b.set_collision_shape(sx.CollisionShape.box((0.5, 0.5, 0.5)))

    assert body_a.has_collision_shape
    assert body_a.collision_shape.type == sx.CollisionShapeType.SPHERE
    assert body_b.collision_shape.type == sx.CollisionShapeType.BOX
    assert not body_b.is_deformable_surface_ccd_obstacle
    body_b.is_deformable_surface_ccd_obstacle = True
    assert body_b.is_deformable_surface_ccd_obstacle
    body_b.is_deformable_surface_ccd_obstacle = False
    assert not body_b.is_deformable_surface_ccd_obstacle
    assert not body_b.is_deformable_ground_barrier
    body_b.is_deformable_ground_barrier = True
    assert body_b.is_deformable_ground_barrier
    body_b.is_deformable_ground_barrier = False
    assert not body_b.is_deformable_ground_barrier
    assert world.add_rigid_body("c").collision_shape is None

    contacts = world.collide()
    assert len(contacts) >= 1
    for contact in contacts:
        assert contact.depth > 0.0
        names = {contact.body_a.name, contact.body_b.name}
        assert names == {"a", "b"}

    body_b.transform = _translation_transform(10.0, 0.0, 0.0)
    assert len(world.collide()) == 0


def test_experimental_collision_shape_local_transform():
    sx = _simulation_experimental()

    world = sx.World()

    body_a = world.add_rigid_body("a")
    body_a.set_collision_shape(
        sx.CollisionShape.sphere(0.5, _translation_transform(1.0, 0.0, 0.0))
    )
    body_b = world.add_rigid_body("b", position=(1.8, 0.0, 0.0))
    body_b.set_collision_shape(sx.CollisionShape.sphere(0.5))

    assert np.allclose(
        np.array(body_a.collision_shape.local_transform),
        np.array(_translation_transform(1.0, 0.0, 0.0)),
    )
    contacts = world.collide()
    assert len(contacts) >= 1
    assert contacts[0].depth == pytest.approx(0.2, abs=1e-6)


def test_experimental_compound_collision_shapes():
    sx = _simulation_experimental()

    world = sx.World()

    compound = world.add_rigid_body("compound")
    compound.add_collision_shape(sx.CollisionShape.sphere(0.5))
    compound.add_collision_shape(sx.CollisionShape.sphere(0.5))
    compound.add_collision_shape(
        sx.CollisionShape.sphere(0.5, _translation_transform(2.0, 0.0, 0.0))
    )

    assert compound.has_collision_shape
    assert compound.collision_shape.type == sx.CollisionShapeType.SPHERE
    assert len(compound.collision_shapes) == 3
    assert len(world.collide()) == 0

    target = world.add_rigid_body("target", position=(2.8, 0.0, 0.0))
    target.set_collision_shape(sx.CollisionShape.sphere(0.5))
    contacts = world.collide()
    assert len(contacts) >= 1
    contact = contacts[0]
    if contact.body_a.name == "compound":
        assert contact.shape_index_a == 2
        assert np.isfinite(contact.local_point_a).all()
    else:
        assert contact.body_b.name == "compound"
        assert contact.shape_index_b == 2
        assert np.isfinite(contact.local_point_b).all()

    compound.set_collision_shape(sx.CollisionShape.box((0.1, 0.2, 0.3)))
    assert len(compound.collision_shapes) == 1
    assert compound.collision_shape.type == sx.CollisionShapeType.BOX

    link_world = sx.World()
    robot = link_world.add_multibody("robot")
    base = robot.add_link("base")
    base.add_collision_shape(sx.CollisionShape.sphere(0.25))
    base.add_collision_shape(
        sx.CollisionShape.box(
            (0.1, 0.2, 0.3), _translation_transform(1.0, 0.0, 0.0)
        )
    )
    assert base.has_collision_shape
    assert len(base.collision_shapes) == 2
    assert base.collision_shape.type == sx.CollisionShapeType.SPHERE


def test_experimental_collision_shape_capsule():
    sx = _simulation_experimental()

    world = sx.World()

    capsule = world.add_rigid_body("capsule")
    shape = sx.CollisionShape.capsule(radius=0.25, half_height=0.5)
    capsule.set_collision_shape(shape)
    sphere = world.add_rigid_body("sphere", position=(0.45, 0.0, 0.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.25))

    assert capsule.collision_shape.type == sx.CollisionShapeType.CAPSULE
    assert capsule.collision_shape.radius == pytest.approx(0.25)
    assert capsule.collision_shape.height == pytest.approx(1.0)
    assert capsule.collision_shape.half_height == pytest.approx(0.5)
    assert capsule.collision_shape.half_extents.tolist() == pytest.approx(
        [0.25, 0.25, 0.5]
    )

    contacts = world.collide()
    assert len(contacts) >= 1
    assert contacts[0].depth == pytest.approx(0.05, abs=1e-6)


def test_experimental_collision_shape_cylinder():
    sx = _simulation_experimental()

    world = sx.World()

    cylinder = world.add_rigid_body("cylinder")
    shape = sx.CollisionShape.cylinder(radius=0.25, half_height=0.5)
    cylinder.set_collision_shape(shape)
    sphere = world.add_rigid_body("sphere", position=(0.45, 0.0, 0.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.25))

    assert cylinder.collision_shape.type == sx.CollisionShapeType.CYLINDER
    assert cylinder.collision_shape.radius == pytest.approx(0.25)
    assert cylinder.collision_shape.height == pytest.approx(1.0)
    assert cylinder.collision_shape.half_height == pytest.approx(0.5)
    assert cylinder.collision_shape.half_extents.tolist() == pytest.approx(
        [0.25, 0.25, 0.5]
    )

    contacts = world.collide()
    assert len(contacts) >= 1
    assert contacts[0].depth == pytest.approx(0.05, abs=1e-6)


def test_experimental_collision_shape_plane():
    sx = _simulation_experimental()

    world = sx.World()

    plane = world.add_rigid_body("plane")
    shape = sx.CollisionShape.plane((0.0, 0.0, 1.0), 0.0)
    plane.set_collision_shape(shape)
    sphere = world.add_rigid_body("sphere", position=(0.0, 0.0, 0.2))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.25))

    assert plane.collision_shape.type == sx.CollisionShapeType.PLANE
    np.testing.assert_allclose(plane.collision_shape.normal, (0.0, 0.0, 1.0))
    assert plane.collision_shape.offset == pytest.approx(0.0)

    contacts = world.collide()
    assert len(contacts) >= 1
    assert contacts[0].depth == pytest.approx(0.05, abs=1e-6)


def test_experimental_collision_shape_mesh():
    sx = _simulation_experimental()

    world = sx.World()

    mesh = world.add_rigid_body("mesh")
    shape = sx.CollisionShape.mesh(
        [
            (-1.0, -1.0, 0.0),
            (1.0, -1.0, 0.0),
            (-1.0, 1.0, 0.0),
            (1.0, 1.0, 0.0),
        ],
        [(0, 1, 2), (1, 3, 2)],
    )
    mesh.set_collision_shape(shape)
    sphere = world.add_rigid_body("sphere", position=(0.0, 0.0, 0.2))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.25))

    assert mesh.collision_shape.type == sx.CollisionShapeType.MESH
    assert len(mesh.collision_shape.vertices) == 4
    assert len(mesh.collision_shape.triangles) == 2

    contacts = world.collide()
    assert len(contacts) >= 1
    assert contacts[0].depth == pytest.approx(0.05, abs=1e-6)


def test_experimental_mesh_collision_shape_public_surface():
    sx = _simulation_experimental()

    shape = sx.CollisionShape.mesh(
        vertices=[
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        ],
        triangles=[(0, 1, 2)],
    )

    assert shape.type == sx.CollisionShapeType.MESH
    np.testing.assert_allclose(
        [vertex.tolist() for vertex in shape.vertices],
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
    )
    assert [triangle.tolist() for triangle in shape.triangles] == [[0, 1, 2]]

    world = sx.World()
    body = world.add_rigid_body("mesh")
    body.set_collision_shape(shape)
    assert body.collision_shape.type == sx.CollisionShapeType.MESH


def test_experimental_mesh_collision_shape_drives_rigid_ipc_body():
    sx = _simulation_experimental()

    world = sx.World()

    # A unit tetrahedron mesh authored from Python vertex/triangle lists.
    vertices = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    ]
    triangles = [
        (0, 2, 1),
        (0, 1, 3),
        (0, 3, 2),
        (1, 2, 3),
    ]

    body = world.add_rigid_body("tet", position=(0.0, 0.0, 0.0))
    body.set_collision_shape(sx.CollisionShape.mesh(vertices, triangles))

    assert body.has_collision_shape
    assert body.collision_shape.type == sx.CollisionShapeType.MESH

    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.gravity = (0.0, 0.0, 0.0)
    body.force = (2.0, 0.0, 0.0)
    world.enter_simulation_mode()
    world.step()
    assert body.transform[0, 3] > 0.0


def test_experimental_cylinder_collision_shape_public_surface_and_query():
    sx = _simulation_experimental()

    shape = sx.CollisionShape.cylinder(radius=0.25, half_height=0.75)
    assert shape.type == sx.CollisionShapeType.CYLINDER
    assert shape.radius == pytest.approx(0.25)
    assert shape.half_extents.tolist() == pytest.approx([0.25, 0.25, 0.75])

    world = sx.World()
    cylinder = world.add_rigid_body("cylinder")
    cylinder.set_collision_shape(shape)
    sphere = world.add_rigid_body("sphere", position=(0.4, 0.0, 0.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.25))

    contacts = world.collide()
    assert len(contacts) >= 1
    for contact in contacts:
        names = {contact.body_a.name, contact.body_b.name}
        assert names == {"cylinder", "sphere"}


def test_experimental_kinematic_body():
    sx = _simulation_experimental()

    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.gravity = (0.0, 0.0, -9.81)
    world.time_step = 0.1

    body = world.add_rigid_body("kinematic", position=(0.0, 0.0, 0.0))
    body.set_collision_shape(sx.CollisionShape.box((0.2, 0.2, 0.2)))
    body.linear_velocity = (1.0, 0.0, 0.0)

    assert not body.is_kinematic
    body.is_kinematic = True
    assert body.is_kinematic
    assert not body.is_static

    world.enter_simulation_mode()
    world.step()

    # Advanced by its prescribed velocity (ignoring gravity), velocity preserved.
    assert body.transform[0, 3] == pytest.approx(0.1, abs=1e-9)
    assert body.transform[2, 3] == pytest.approx(0.0, abs=1e-9)
    np.testing.assert_allclose(body.linear_velocity, [1.0, 0.0, 0.0], atol=1e-9)


def test_experimental_static_setter_clears_kinematic_body():
    sx = _simulation_experimental()

    world = sx.World()
    world.rigid_body_solver = sx.RigidBodySolver.IPC
    world.gravity = (0.0, 0.0, 0.0)
    world.time_step = 0.1

    body = world.add_rigid_body("body", position=(0.0, 0.0, 0.0))
    body.set_collision_shape(sx.CollisionShape.box((0.2, 0.2, 0.2)))
    body.linear_velocity = (1.0, 0.0, 0.0)
    body.is_kinematic = True
    assert body.is_kinematic
    assert not body.is_static

    body.is_static = True
    assert body.is_static
    assert not body.is_kinematic

    world.enter_simulation_mode()
    world.step()

    assert body.transform[0, 3] == pytest.approx(0.0, abs=1e-9)
    np.testing.assert_allclose(body.linear_velocity, [1.0, 0.0, 0.0], atol=1e-9)


def test_experimental_collision_query_includes_links():
    sx = _simulation_experimental()

    world = sx.World()

    robot = world.add_multibody("robot")
    base = robot.add_link("base")
    base.set_collision_shape(sx.CollisionShape.sphere(1.0))
    assert base.has_collision_shape
    assert base.collision_shape.type == sx.CollisionShapeType.SPHERE

    ball = world.add_rigid_body("ball", position=(1.2, 0.0, 0.0))
    ball.set_collision_shape(sx.CollisionShape.sphere(0.5))

    world.enter_simulation_mode()
    contacts = world.collide()
    assert len(contacts) >= 1

    saw_link = False
    saw_rigid_body = False
    for contact in contacts:
        assert contact.depth > 0.0
        saw_link = saw_link or contact.body_a.is_link or contact.body_b.is_link
        saw_rigid_body = (
            saw_rigid_body
            or contact.body_a.is_rigid_body
            or contact.body_b.is_rigid_body
        )
        for body in (contact.body_a, contact.body_b):
            if body.is_link:
                assert body.name == "base"
                assert body.as_link() is not None
                assert body.as_rigid_body() is None
    assert saw_link
    assert saw_rigid_body


def test_experimental_collision_query_can_filter_same_multibody_link_pairs():
    sx = _simulation_experimental()

    world = sx.World()

    robot = world.add_multibody("robot")
    base = robot.add_link("base")
    base.set_collision_shape(sx.CollisionShape.sphere(0.75))

    link = robot.add_link(
        "link",
        parent=base,
        joint=sx.JointSpec(name="slider", type=sx.JointType.PRISMATIC),
    )
    link.set_collision_shape(sx.CollisionShape.sphere(0.75))

    obstacle = world.add_rigid_body("obstacle", position=(10.0, 0.0, 0.0))
    obstacle.set_collision_shape(sx.CollisionShape.sphere(0.25))

    world.enter_simulation_mode()

    default_contacts = world.collide()
    assert any(
        contact.body_a.is_link
        and contact.body_b.is_link
        and {contact.body_a.name, contact.body_b.name} == {"base", "link"}
        for contact in default_contacts
    )

    options = sx.CollisionQueryOptions(include_same_multibody_link_pairs=False)
    assert len(world.collide(options)) == 0
    assert "include_same_multibody_link_pairs=False" in repr(options)

    obstacle.transform = _translation_transform(0.2, 0.0, 0.0)

    filtered_contacts = world.collide(options)
    assert len(filtered_contacts) >= 1
    assert all(
        contact.body_a.is_rigid_body or contact.body_b.is_rigid_body
        for contact in filtered_contacts
    )


def test_experimental_collision_query_can_filter_body_type_pairs():
    sx = _simulation_experimental()

    world = sx.World()
    body_a = world.add_rigid_body("rigid_a")
    body_a.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body_b = world.add_rigid_body("rigid_b", position=(0.4, 0.0, 0.0))
    body_b.set_collision_shape(sx.CollisionShape.sphere(0.5))

    assert len(world.collide()) >= 1

    options = sx.CollisionQueryOptions(include_rigid_body_pairs=False)
    assert len(world.collide(options)) == 0
    assert "include_rigid_body_pairs=False" in repr(options)

    world = sx.World()
    robot = world.add_multibody("robot")
    link = robot.add_link("link")
    link.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body = world.add_rigid_body("rigid", position=(0.4, 0.0, 0.0))
    body.set_collision_shape(sx.CollisionShape.sphere(0.5))

    world.enter_simulation_mode()
    assert len(world.collide()) >= 1

    options = sx.CollisionQueryOptions(include_rigid_body_link_pairs=False)
    assert len(world.collide(options)) == 0
    assert "include_rigid_body_link_pairs=False" in repr(options)

    world = sx.World()
    robot = world.add_multibody("robot")
    base = robot.add_link("base")
    base.set_collision_shape(sx.CollisionShape.sphere(0.5))
    child = robot.add_link(
        "child",
        parent=base,
        joint=sx.JointSpec(name="child", type=sx.JointType.FIXED),
    )
    child.set_collision_shape(sx.CollisionShape.sphere(0.5))

    world.enter_simulation_mode()
    assert len(world.collide()) >= 1

    options = sx.CollisionQueryOptions(include_link_pairs=False)
    assert len(world.collide(options)) == 0
    assert "include_link_pairs=False" in repr(options)


def test_experimental_contact_stops_approaching_bodies():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)

    body_a = world.add_rigid_body(
        "a", position=(-0.45, 0.0, 0.0), linear_velocity=(1.0, 0.0, 0.0)
    )
    body_a.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body_b = world.add_rigid_body(
        "b", position=(0.45, 0.0, 0.0), linear_velocity=(-1.0, 0.0, 0.0)
    )
    body_b.set_collision_shape(sx.CollisionShape.sphere(0.5))

    world.time_step = 0.001
    world.step()

    # Equal-mass head-on fully inelastic contact: both come to rest.
    assert body_a.linear_velocity.tolist()[0] == pytest.approx(0.0, abs=1e-9)
    assert body_b.linear_velocity.tolist()[0] == pytest.approx(0.0, abs=1e-9)


def test_experimental_body_rests_on_static_ground():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.5)))
    assert ground.is_static

    sphere = world.add_rigid_body("sphere", position=(0.0, 0.0, 2.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.5))
    assert not sphere.is_static

    world.time_step = 0.005
    world.step(n=1000)

    # Sphere (radius 0.5) rests on the ground (top at z = 0) and stops.
    assert sphere.translation.tolist()[2] == pytest.approx(0.5, abs=2e-2)
    assert abs(sphere.linear_velocity.tolist()[2]) < 0.1
    assert ground.translation.tolist() == pytest.approx([0.0, 0.0, -0.5])


def test_experimental_multibody_link_rests_on_static_ground():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    robot = world.add_multibody("leg_robot")
    base = robot.add_link("base")
    leg = robot.add_link(
        "leg",
        parent=base,
        joint=sx.JointSpec(
            name="slider", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    leg.mass = 1.0
    leg.set_collision_shape(sx.CollisionShape.sphere(0.2))

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -1.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.5)))

    joint = leg.parent_joint
    joint.position = [-0.25]  # start just above the ground

    world.time_step = 0.002
    world.step(n=1500)

    # The sphere (radius 0.2) rests on the ground top (z = -0.5): leg origin at
    # z = -0.3 with near-zero velocity and no deep penetration.
    rest_z = leg.transform[2, 3]
    assert rest_z == pytest.approx(-0.3, abs=5e-3)
    assert rest_z > -0.31
    assert joint.velocity.tolist()[0] == pytest.approx(0.0, abs=5e-2)


def test_experimental_multibody_link_pushes_dynamic_rigid_body():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)

    # Striker: a prismatic-X link moving toward a free rigid body.
    robot = world.add_multibody("striker_robot")
    base = robot.add_link("base")
    striker = robot.add_link(
        "striker",
        parent=base,
        joint=sx.JointSpec(
            name="rail", type=sx.JointType.PRISMATIC, axis=(1.0, 0.0, 0.0)
        ),
    )
    striker_mass = 2.0
    striker.mass = striker_mass
    striker.set_collision_shape(sx.CollisionShape.sphere(0.2))
    joint = striker.parent_joint
    initial_speed = 1.0
    joint.velocity = [initial_speed]

    box_mass = 1.0
    box = world.add_rigid_body("box", mass=box_mass, position=(0.5, 0.0, 0.0))
    box.set_collision_shape(sx.CollisionShape.sphere(0.2))

    world.time_step = 0.002
    world.enter_simulation_mode()

    initial_momentum = striker_mass * initial_speed
    box_pushed = False
    for _ in range(600):
        world.step()
        momentum = striker_mass * joint.velocity[0] + box_mass * box.linear_velocity[0]
        # Equal-and-opposite contact impulses conserve total X momentum.
        assert momentum == pytest.approx(initial_momentum, abs=1e-6)
        if box.linear_velocity[0] > 1e-3:
            box_pushed = True

    assert box_pushed
    assert box.linear_velocity[0] > 0.1
    assert joint.velocity[0] < initial_speed
    common_velocity = initial_momentum / (striker_mass + box_mass)
    assert box.linear_velocity[0] == pytest.approx(common_velocity, abs=0.1)


def test_experimental_multibody_link_contact_friction_stops_slide():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    robot = world.add_multibody("slider_robot")
    base = robot.add_link("base")
    carrier = robot.add_link(
        "carrier",
        parent=base,
        joint=sx.JointSpec(
            name="vertical", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    carrier.mass = 0.1
    slider = robot.add_link(
        "slider",
        parent=carrier,
        joint=sx.JointSpec(
            name="horizontal", type=sx.JointType.PRISMATIC, axis=(1.0, 0.0, 0.0)
        ),
    )
    slider.mass = 1.0
    slider.set_collision_shape(sx.CollisionShape.sphere(0.2))

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -1.0))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((20.0, 20.0, 0.5)))

    vertical = carrier.parent_joint
    horizontal = slider.parent_joint
    vertical.position = [-0.3]  # rest height (sphere on ground top z = -0.5)
    horizontal.velocity = [1.0]

    world.time_step = 0.002
    world.step(n=600)

    # Friction brakes the slide to rest; the slider advanced but did not reverse,
    # and stayed resting on the ground.
    assert horizontal.velocity.tolist()[0] == pytest.approx(0.0, abs=5e-2)
    assert horizontal.position.tolist()[0] > 0.01
    assert slider.transform[2, 3] == pytest.approx(-0.3, abs=1e-2)


def test_experimental_multibody_link_contact_restitution_bounces():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    robot = world.add_multibody("bouncer")
    base = robot.add_link("base")
    bob = robot.add_link(
        "bob",
        parent=base,
        joint=sx.JointSpec(
            name="slider", type=sx.JointType.PRISMATIC, axis=(0.0, 0.0, 1.0)
        ),
    )
    bob.mass = 1.0
    bob.set_collision_shape(sx.CollisionShape.sphere(0.2))

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -1.0))
    ground.is_static = True
    ground.restitution = 0.9
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.5)))

    joint = bob.parent_joint
    joint.position = [0.0]  # drop 0.3 m to contact

    world.time_step = 0.002
    max_upward_velocity = 0.0
    for _ in range(400):
        world.step()
        max_upward_velocity = max(max_upward_velocity, joint.velocity.tolist()[0])

    # Impact speed ~2.4 m/s; e = 0.9 rebounds at ~2.2 m/s.
    assert max_upward_velocity > 1.5


def test_experimental_contact_restitution():
    sx = _simulation_experimental()

    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)

    body_a = world.add_rigid_body(
        "a", position=(-0.45, 0.0, 0.0), linear_velocity=(1.0, 0.0, 0.0)
    )
    body_a.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body_a.restitution = 1.0
    body_b = world.add_rigid_body(
        "b", position=(0.45, 0.0, 0.0), linear_velocity=(-1.0, 0.0, 0.0)
    )
    body_b.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body_b.restitution = 1.0

    assert body_a.restitution == pytest.approx(1.0)
    assert world.add_rigid_body("c").restitution == pytest.approx(0.0)
    assert world.add_rigid_body("d").friction == pytest.approx(1.0)

    world.time_step = 0.001
    world.step()

    # Perfectly elastic equal-mass head-on collision swaps velocities.
    assert body_a.linear_velocity.tolist()[0] == pytest.approx(-1.0, abs=1e-9)
    assert body_b.linear_velocity.tolist()[0] == pytest.approx(1.0, abs=1e-9)


def test_experimental_contact_friction():
    sx = _simulation_experimental()

    world = sx.World()  # default gravity (0, 0, -9.81)

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.5)))

    slider = world.add_rigid_body(
        "slider", position=(0.0, 0.0, 0.1), linear_velocity=(2.0, 0.0, 0.0)
    )
    slider.set_collision_shape(sx.CollisionShape.box((0.5, 0.5, 0.1)))
    slider.friction = 0.5
    assert slider.friction == pytest.approx(0.5)

    world.time_step = 0.005
    world.step(n=400)

    # Friction brakes the slide without reversing it.
    velocity_x = slider.linear_velocity.tolist()[0]
    assert velocity_x < 0.5
    assert velocity_x > -0.2
    assert slider.translation.tolist()[0] > 0.0


def test_experimental_rigid_body_options_reject_invalid_values():
    sx = _simulation_experimental()

    with pytest.raises(Exception, match="mass must be positive and finite"):
        sx.RigidBodyOptions(mass=0.0)

    with pytest.raises(Exception, match="inertia must be symmetric positive definite"):
        sx.RigidBodyOptions(
            inertia=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
        )

    with pytest.raises(Exception, match="position must contain only finite"):
        sx.RigidBodyOptions(position=(math.inf, 0.0, 0.0))

    with pytest.raises(Exception, match="orientation must be finite and non-zero"):
        sx.RigidBodyOptions(orientation=(0.0, 0.0, 0.0, 0.0))

    options = sx.RigidBodyOptions()

    with pytest.raises(Exception, match="linear_velocity must contain only finite"):
        options.linear_velocity = (math.nan, 0.0, 0.0)

    with pytest.raises(Exception, match="angular_velocity must contain only finite"):
        options.angular_velocity = (0.0, math.inf, 0.0)

    with pytest.raises(Exception, match="JointSpec.axis must be non-zero"):
        sx.JointSpec(axis=(0.0, 0.0, 0.0))

    with pytest.raises(Exception, match="JointSpec.axis must contain only finite"):
        sx.JointSpec(axis=(math.nan, 0.0, 0.0))

    joint_spec = sx.JointSpec()
    with pytest.raises(Exception, match="JointSpec.axis must contain only finite"):
        joint_spec.axis = (0.0, math.inf, 0.0)

    world = sx.World()
    with pytest.raises(Exception, match="mass must be positive and finite"):
        world.add_rigid_body("box", mass=math.inf)

    body = world.add_rigid_body("body")
    with pytest.raises(Exception, match="RigidBody linear velocity"):
        body.linear_velocity = (math.nan, 0.0, 0.0)
    with pytest.raises(Exception, match="RigidBody angular velocity"):
        body.angular_velocity = (0.0, math.inf, 0.0)
    with pytest.raises(Exception, match="RigidBody mass"):
        body.mass = 0.0
    with pytest.raises(Exception, match="RigidBody inertia"):
        body.inertia = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    with pytest.raises(Exception, match="RigidBody force"):
        body.force = (math.nan, 0.0, 0.0)
    with pytest.raises(Exception, match="RigidBody force"):
        body.apply_force((0.0, math.inf, 0.0))
    with pytest.raises(Exception, match="RigidBody torque"):
        body.torque = (0.0, math.inf, 0.0)
    with pytest.raises(Exception, match="RigidBody torque"):
        body.apply_torque((0.0, 0.0, math.nan))

    arm = world.add_multibody("arm")
    base = arm.add_link("base")
    link = arm.add_link("link", parent=base, joint=sx.JointSpec(name="joint"))
    joint = link.parent_joint
    with pytest.raises(Exception, match="Joint position dimension"):
        joint.position = [0.0, 1.0]
    with pytest.raises(Exception, match="Joint position"):
        joint.position = [math.inf]
    with pytest.raises(Exception, match="Joint velocity dimension"):
        joint.velocity = []
    with pytest.raises(Exception, match="Joint velocity"):
        joint.velocity = [math.nan]
    with pytest.raises(Exception):
        joint.position = 0.0
    with pytest.raises(Exception, match="1-D vector"):
        joint.velocity = np.zeros((1, 1), dtype=float)


def test_experimental_loop_closure_rejects_invalid_topology():
    sx = _simulation_experimental()

    world = sx.World()
    arm = world.add_multibody("arm")
    base = arm.add_link("base")
    link = arm.add_link("link", parent=base, joint=sx.JointSpec(name="joint"))

    other_world = sx.World()
    other_body = other_world.add_rigid_body("other")

    with pytest.raises(Exception, match="distinct frames"):
        world.add_loop_closure("same", frame_a=base, frame_b=base)

    with pytest.raises(Exception, match="different world"):
        world.add_loop_closure("cross_world", frame_a=base, frame_b=other_body)

    bad_offset = np.asarray(_translation_transform(0.0, 0.0, 0.0), dtype=float)
    bad_offset[0, 0] = math.nan
    with pytest.raises(Exception, match="finite values"):
        sx.LoopClosureSpec(frame_a=base, frame_b=link, offset_a=bad_offset)

    world.add_loop_closure("valid", frame_a=base, frame_b=link)
    with pytest.raises(Exception, match="already exists"):
        world.add_loop_closure("valid", frame_a=link, frame_b=base)

    world.enter_simulation_mode()
    with pytest.raises(Exception, match="simulation mode"):
        world.add_loop_closure("after_bake", frame_a=base, frame_b=link)


def test_experimental_loop_closure_rejects_unsupported_active_policy():
    sx = _simulation_experimental()

    world = sx.World()
    arm = world.add_multibody("arm")
    base = arm.add_link("base")
    link = arm.add_link("link", parent=base, joint=sx.JointSpec(name="joint"))
    closure = world.add_loop_closure("closure", frame_a=base, frame_b=link)

    closure.kinematics = sx.ClosureKinematicsPolicy.PROJECT
    with pytest.raises(Exception, match="projection stage"):
        world.enter_simulation_mode()
    assert not world.is_simulation_mode

    closure.enabled = False
    closure.dynamics = sx.ClosureDynamicsPolicy.SOLVE
    world.enter_simulation_mode()
    world.sync()
    world.step()

    closure.enabled = True
    with pytest.raises(Exception, match="projection stage"):
        world.sync(sx.WorldSyncStage.KINEMATICS)

    closure.kinematics = sx.ClosureKinematicsPolicy.RESIDUAL_ONLY
    with pytest.raises(Exception, match="solving stage"):
        world.step()


def test_experimental_deformable_body_python_api():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.01)

    options = sx.DeformableBodyOptions()
    options.positions = [np.array([0.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0])]
    options.masses = [1.0, 1.0]
    options.edges = [sx.DeformableEdge(0, 1, 1.0)]
    options.fixed_nodes = [0]
    options.edge_stiffness = 100.0
    material = sx.DeformableMaterialProperties()
    material.friction_coefficient = 0.5
    options.material = material

    body = world.add_deformable_body("strand", options)
    assert body.is_valid
    assert body.name == "strand"
    assert body.node_count == 2
    assert body.edge_count == 1
    assert world.num_deformable_bodies == 1
    assert world.has_deformable_body("strand")
    assert not world.has_deformable_body("missing")
    assert world.get_deformable_body("missing") is None

    assert body.is_fixed_node(0)
    assert not body.is_fixed_node(1)
    assert body.material_properties.friction_coefficient == pytest.approx(0.5)

    edge = body.edge(0)
    assert edge.node_a == 0
    assert edge.node_b == 1

    # The fixed node stays put; the stretched spring contracts the free node.
    np.testing.assert_allclose(body.node_position(0), [0.0, 0.0, 0.0], atol=1e-12)
    before_x = body.node_position(1)[0]
    world.step()
    after_x = body.node_position(1)[0]
    assert after_x < before_x

    fetched = world.get_deformable_body("strand")
    assert fetched is not None
    assert fetched.node_count == 2


def test_experimental_deformable_body_topology_python_api():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.01)

    # A single unit-corner tetrahedron with an explicit boundary surface.
    options = sx.DeformableBodyOptions()
    options.positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    options.masses = [1.0, 1.0, 1.0, 1.0]
    options.tetrahedra = [sx.DeformableTetrahedron(0, 1, 2, 3)]
    options.surface_triangles = [
        sx.DeformableSurfaceTriangle(0, 2, 1),
        sx.DeformableSurfaceTriangle(0, 1, 3),
        sx.DeformableSurfaceTriangle(0, 3, 2),
        sx.DeformableSurfaceTriangle(1, 2, 3),
    ]

    body = world.add_deformable_body("tet", options)
    assert body.is_valid
    assert body.node_count == 4
    assert body.surface_triangle_count == 4
    assert body.tetrahedron_count == 1

    triangle = body.surface_triangle(0)
    assert (triangle.node_a, triangle.node_b, triangle.node_c) == (0, 2, 1)

    tetrahedron = body.tetrahedron(0)
    assert (
        tetrahedron.node_a,
        tetrahedron.node_b,
        tetrahedron.node_c,
        tetrahedron.node_d,
    ) == (0, 1, 2, 3)

    # The unit corner tetrahedron has volume 1/6.
    assert body.tetrahedron_rest_volume(0) == pytest.approx(1.0 / 6.0)
    assert body.node_mass(0) == pytest.approx(1.0)


def test_experimental_adaptive_barrier_stiffness_holds_heavy_node_higher():
    sx = _simulation_experimental()

    def settle(adaptive: bool) -> float:
        world = sx.World(time_step=0.01)
        world.gravity = [0.0, 0.0, -0.5]
        ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box((10.0, 10.0, 0.5)))
        ground.is_deformable_ground_barrier = True

        options = sx.DeformableBodyOptions()
        options.positions = [np.array([0.0, 0.0, 0.015])]
        options.masses = [40.0]
        options.material.use_adaptive_barrier_stiffness = adaptive
        body = world.add_deformable_body("heavy_node", options)
        world.step(600)
        return float(body.node_position(0)[2])

    fixed_z = settle(False)
    adaptive_z = settle(True)
    # Both settle intersection-free in the activation band (d_hat = 2e-2).
    assert 0.0 < fixed_z < 2e-2
    assert 0.0 < adaptive_z < 2e-2
    # The stiffer adaptive barrier holds the heavy node measurably higher.
    assert adaptive_z > fixed_z + 1e-3


def test_experimental_world_exposes_deformable_solver_diagnostics():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.01)
    world.gravity = [0.0, 0.0, -9.81]

    options = sx.DeformableBodyOptions()
    options.positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    options.tetrahedra = [sx.DeformableTetrahedron(0, 1, 2, 3)]
    options.material.youngs_modulus = 1.0e4
    options.material.use_finite_element_elasticity = True
    options.fixed_nodes = [0]
    world.add_deformable_body("tet", options)

    # Before any step the diagnostics are zero-initialized.
    before = world.last_deformable_solver_diagnostics
    assert before.body_count == 0
    assert before.node_count == 0
    assert before.solver_iterations == 0
    assert before.projected_newton_hessian_nonzeros == 0
    assert before.projected_newton_hessian_storage_bytes == 0
    assert before.projected_newton_matrix_free_solves == 0

    world.step(5)

    after = world.last_deformable_solver_diagnostics
    assert after.body_count == 1
    assert after.node_count == 4
    # The FEM body runs the implicit projected-Newton solve every step.
    assert after.solver_iterations >= 1
    assert after.objective_evaluations >= 1
    assert after.projected_newton_steps + after.projected_newton_fallbacks >= 1
    assert after.projected_newton_hessian_nonzeros > 0
    assert after.projected_newton_hessian_storage_bytes > 0
    # This body uses the default direct (sparse Cholesky) solve, so the iterative
    # (conjugate-gradient) path is never taken.
    assert after.projected_newton_iterative_solves == 0
    assert after.projected_newton_matrix_free_solves == 0
    assert after.projected_newton_iterative_iterations == 0
    assert after.projected_newton_iterative_max_error == 0.0
    # No contacts in this free-hanging single tet.
    assert after.self_contact_barrier_active_contacts == 0
    assert after.converged_active_contact_count == 0

    # The snapshot is read-only.
    with pytest.raises((AttributeError, TypeError)):
        after.node_count = 99


def test_experimental_world_iterative_solver_diagnostic():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.01)
    world.gravity = [0.0, 0.0, -9.81]

    options = sx.DeformableBodyOptions()
    options.positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    options.tetrahedra = [sx.DeformableTetrahedron(0, 1, 2, 3)]
    options.material.youngs_modulus = 1.0e4
    options.material.use_finite_element_elasticity = True
    # Opt in to the iterative (incomplete-Cholesky-preconditioned CG) linear
    # solve instead of the sparse Cholesky factorization.
    options.material.use_iterative_linear_solver = True
    options.fixed_nodes = [0]
    world.add_deformable_body("tet", options)

    # The iterative-solve count surfaces through the public diagnostics, so a
    # caller can observe which linear-solve path the projected-Newton step took.
    total_iterative_solves = 0
    total_iterative_iterations = 0
    max_hessian_nonzeros = 0
    max_hessian_storage_bytes = 0
    max_iterative_error = 0.0
    for _ in range(8):
        world.step()
        diagnostics = world.last_deformable_solver_diagnostics
        total_iterative_solves += diagnostics.projected_newton_iterative_solves
        total_iterative_iterations += (
            diagnostics.projected_newton_iterative_iterations
        )
        max_hessian_nonzeros = max(
            max_hessian_nonzeros, diagnostics.projected_newton_hessian_nonzeros
        )
        max_hessian_storage_bytes = max(
            max_hessian_storage_bytes,
            diagnostics.projected_newton_hessian_storage_bytes,
        )
        max_iterative_error = max(
            max_iterative_error,
            diagnostics.projected_newton_iterative_max_error,
        )
    assert total_iterative_solves > 0
    assert total_iterative_iterations >= 0
    assert max_hessian_nonzeros > 0
    assert max_hessian_storage_bytes > 0
    assert math.isfinite(max_iterative_error)
    assert max_iterative_error >= 0.0


def test_experimental_world_matrix_free_solver_diagnostic():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.01)
    world.gravity = [0.0, 0.0, -9.81]

    options = sx.DeformableBodyOptions()
    options.positions = [
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    ]
    options.tetrahedra = [sx.DeformableTetrahedron(0, 1, 2, 3)]
    options.material.youngs_modulus = 1.0e4
    options.material.use_finite_element_elasticity = True
    options.material.use_matrix_free_linear_solver = True
    options.fixed_nodes = [0]
    world.add_deformable_body("tet", options)

    total_iterative_solves = 0
    total_matrix_free_solves = 0
    total_iterative_iterations = 0
    max_hessian_nonzeros = 0
    max_hessian_storage_bytes = 0
    max_iterative_error = 0.0
    for _ in range(8):
        world.step()
        diagnostics = world.last_deformable_solver_diagnostics
        total_iterative_solves += diagnostics.projected_newton_iterative_solves
        total_matrix_free_solves += (
            diagnostics.projected_newton_matrix_free_solves
        )
        total_iterative_iterations += (
            diagnostics.projected_newton_iterative_iterations
        )
        max_hessian_nonzeros = max(
            max_hessian_nonzeros, diagnostics.projected_newton_hessian_nonzeros
        )
        max_hessian_storage_bytes = max(
            max_hessian_storage_bytes,
            diagnostics.projected_newton_hessian_storage_bytes,
        )
        max_iterative_error = max(
            max_iterative_error,
            diagnostics.projected_newton_iterative_max_error,
        )

    assert total_matrix_free_solves > 0
    assert total_iterative_solves == total_matrix_free_solves
    assert total_iterative_iterations >= 0
    assert max_hessian_nonzeros == 0
    assert max_hessian_storage_bytes == 0
    assert math.isfinite(max_iterative_error)
    assert max_iterative_error >= 0.0


def test_experimental_world_matrix_free_solver_matches_direct_ground_contact():
    sx = _simulation_experimental()

    def settle(matrix_free: bool):
        world = sx.World(time_step=0.01)
        world.gravity = [0.0, 0.0, -0.5]
        ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box((10.0, 10.0, 0.5)))
        ground.is_deformable_ground_barrier = True

        options = sx.DeformableBodyOptions()
        options.positions = [np.array([0.0, 0.0, 0.015])]
        options.masses = [40.0]
        options.material.use_matrix_free_linear_solver = matrix_free
        body = world.add_deformable_body("node", options)

        total_iterative_solves = 0
        total_matrix_free_solves = 0
        max_hessian_nonzeros = 0
        for _ in range(600):
            world.step()
            diagnostics = world.last_deformable_solver_diagnostics
            total_iterative_solves += (
                diagnostics.projected_newton_iterative_solves
            )
            total_matrix_free_solves += (
                diagnostics.projected_newton_matrix_free_solves
            )
            max_hessian_nonzeros = max(
                max_hessian_nonzeros,
                diagnostics.projected_newton_hessian_nonzeros,
            )
        return (
            float(body.node_position(0)[2]),
            total_iterative_solves,
            total_matrix_free_solves,
            max_hessian_nonzeros,
        )

    direct_z, direct_solves, direct_matrix_free, direct_hessian_nonzeros = (
        settle(False)
    )
    matrix_z, matrix_solves, matrix_free_solves, matrix_hessian_nonzeros = (
        settle(True)
    )

    assert direct_solves == 0
    assert direct_matrix_free == 0
    assert direct_hessian_nonzeros > 0

    assert matrix_solves > 0
    assert matrix_solves == matrix_free_solves
    assert matrix_hessian_nonzeros == 0
    assert 0.0 < matrix_z < 2e-2
    assert matrix_z == pytest.approx(direct_z, abs=1e-9)


def test_experimental_deformable_body_boundary_conditions_python_api():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.1)
    world.gravity = np.array([0.0, 0.0, 0.0])

    options = sx.DeformableBodyOptions()
    options.positions = [
        np.array([0.0, 0.0, 0.0]),  # Dirichlet-scripted
        np.array([1.0, 0.0, 0.0]),  # Neumann-accelerated, free
    ]
    options.masses = [1.0, 1.0]

    dirichlet = sx.DeformableDirichletBoundaryCondition()
    dirichlet.nodes = [0]
    dirichlet.linear_velocity = np.array([1.0, 0.0, 0.0])
    options.dirichlet_boundary_conditions = [dirichlet]

    neumann = sx.DeformableNeumannBoundaryCondition()
    neumann.nodes = [1]
    neumann.acceleration = np.array([0.0, 0.0, -10.0])
    options.neumann_boundary_conditions = [neumann]

    body = world.add_deformable_body("scripted", options)
    assert body.is_valid

    world.step()

    # The Dirichlet node follows the scripted linear motion: x += vx * dt.
    assert body.node_position(0)[0] == pytest.approx(0.1, abs=1e-9)
    # The Neumann node accelerates downward under the applied acceleration.
    assert body.node_position(1)[2] < 0.0


def test_experimental_world_replay_recording_python_api():
    sx = _simulation_experimental()
    world = sx.World(time_step=0.1)
    world.gravity = np.zeros(3)
    body = world.add_rigid_body(
        "body",
        position=np.array([0.0, 0.0, 1.0]),
        linear_velocity=np.array([1.0, 0.0, 0.0]),
    )

    assert world.replay_recording_enabled is False
    assert world.replay_frame_count == 0
    assert world.replay_cursor is None

    world.replay_recording_enabled = True
    assert world.replay_recording_enabled is True
    assert world.replay_frame_count == 1
    assert world.replay_cursor == 0
    assert world.get_replay_frame_time(0) == pytest.approx(0.0)
    assert world.get_replay_simulation_frame(0) == 0

    world.step(3)
    assert world.replay_frame_count == 4
    assert world.replay_cursor == 3
    assert world.time == pytest.approx(0.3)
    assert world.frame == 3
    assert body.translation[0] == pytest.approx(0.3)

    world.restore_replay_frame(1)
    assert world.time == pytest.approx(0.1)
    assert world.frame == 1
    assert body.translation.tolist() == pytest.approx([0.1, 0.0, 1.0])
    assert body.linear_velocity.tolist() == pytest.approx([1.0, 0.0, 0.0])

    world.step()
    assert world.replay_frame_count == 3
    assert world.replay_cursor == 2
    assert world.get_replay_simulation_frame(2) == 2
    assert world.get_replay_frame_time(2) == pytest.approx(0.2)
    assert body.translation[0] == pytest.approx(0.2)

    world.clear_replay_recording()
    assert world.replay_frame_count == 1
    assert world.replay_cursor == 0

    world.replay_recording_enabled = False
    world.step()
    assert world.replay_frame_count == 1

    with pytest.raises(Exception):
        world.restore_replay_frame(-1)


def test_experimental_deformable_scene_loader_python_api(tmp_path):
    sx = _simulation_experimental()

    # A minimal single-tetrahedron Gmsh 4.1 mesh (no explicit surface section,
    # so the loader derives the four boundary faces).
    mesh_dir = tmp_path / "input" / "tetMeshes"
    mesh_dir.mkdir(parents=True)
    (mesh_dir / "tet.msh").write_text(
        "$MeshFormat\n"
        "4.1 0 8\n"
        "$EndMeshFormat\n"
        "$Nodes\n"
        "1 4 1 4\n"
        "3 0 0 4\n"
        "1\n2\n3\n4\n"
        "0.000000e+00 0.000000e+00 0.000000e+00\n"
        "1.000000e+00 0.000000e+00 0.000000e+00\n"
        "0.000000e+00 1.000000e+00 0.000000e+00\n"
        "0.000000e+00 0.000000e+00 1.000000e+00\n"
        "$EndNodes\n"
        "$Elements\n"
        "1 1 1 1\n"
        "3 0 4 1\n"
        "1 1 2 3 4\n"
        "$EndElements\n"
    )

    scene_path = tmp_path / "scene.txt"
    scene_path.write_text(
        "time 0.5 0.1\n"
        "shapes input 1\n"
        "input/tetMeshes/tet.msh 0 0 0  0 0 0  1 1 1 material 6 100 0.2\n"
    )

    world = sx.World()
    options = sx.DeformableSceneLoadOptions()
    options.asset_root = str(tmp_path)
    info = sx.load_deformable_scene(world, str(scene_path), options)

    assert len(info.bodies) == 1
    body_info = info.bodies[0]
    assert body_info.node_count == 4
    assert body_info.tetrahedron_count == 1
    assert body_info.surface_triangle_count == 4
    assert body_info.body.node_count == 4

    diagnostics = sx.collect_deformable_scene_diagnostics(world)
    assert diagnostics.body_count == 1
    assert diagnostics.node_count == 4
    assert diagnostics.tetrahedron_count == 1
    # Unit corner tetrahedron volume 1/6 at density 6 has total mass 1.
    assert diagnostics.total_mass == pytest.approx(1.0)
