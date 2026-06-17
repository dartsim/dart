from __future__ import annotations

import gc
import importlib
import math
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest


def _simulation():
    try:
        module = importlib.import_module("dartpy")
    except ModuleNotFoundError as exc:
        raise AssertionError(
            "dartpy should be available with the DART 7 World stack"
        ) from exc
    if not hasattr(module, "World"):
        raise AssertionError("dartpy imported but did not expose World")
    return module


def _translation_transform(x: float, y: float, z: float):
    return (
        (1.0, 0.0, 0.0, x),
        (0.0, 1.0, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _floating_link_world(sx, name: str):
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody(name)
    base = arm.add_link("base")
    body = arm.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 2.0
    body.inertia = ((0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.3))
    return world, base, body


def _floating_link_pair_world(sx, name: str):
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody(name)
    base = arm.add_link("base")
    parent = arm.add_link(
        "parent",
        parent=base,
        joint=sx.JointSpec(name="parent_float", type=sx.JointType.FLOATING),
    )
    parent.mass = 2.0
    parent.inertia = ((0.2, 0.0, 0.0), (0.0, 0.25, 0.0), (0.0, 0.0, 0.3))
    child = arm.add_link(
        "child",
        parent=base,
        joint=sx.JointSpec(name="child_float", type=sx.JointType.FLOATING),
    )
    child.mass = 1.5
    child.inertia = ((0.15, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.25))
    return world, parent, child


def test_simulation_world_module_is_separate_from_legacy_simulation():
    sx = _simulation()

    assert dart is sx
    assert sx is not dart.simulation
    assert not hasattr(dart, "next")


def test_simulation_api_exposes_python_names_only():
    sx = _simulation()

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
            "isSleeping",
            "getDeactivationGroupIndex",
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
            "applyLinearImpulse",
            "applyAngularImpulse",
            "isSleeping",
            "getDeactivationGroupIndex",
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
            "addArticulatedFixedJoint",
            "addArticulatedRevoluteJoint",
            "addArticulatedPrismaticJoint",
            "addArticulatedSphericalJoint",
            "getArticulatedJoint",
            "hasArticulatedJoint",
            "getArticulatedJoints",
            "getArticulatedJointCount",
            "addRigidBody",
            "addRigidBodyFixedJoint",
            "addRigidBodyRevoluteJoint",
            "addRigidBodyPrismaticJoint",
            "addRigidBodySphericalJoint",
            "addRigidBodyDistanceSpring",
            "hasRigidBodyDistanceSpring",
            "getRigidBodyDistanceSpringParameters",
            "setRigidBodyDistanceSpringParameters",
            "getRigidBodyJoint",
            "hasRigidBodyJoint",
            "getRigidBodyJoints",
            "getRigidBodyJointCount",
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
            "setDeactivationOptions",
            "getDeactivationOptions",
            "isDeactivationEnabled",
            "get_multibody_count",
            "get_loop_closure_count",
            "get_rigid_body_count",
            "updateKinematics",
            "enterSimulationMode",
            "saveBinary",
            "loadBinary",
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


def test_simulation_world_memory_diagnostics_expose_allocator_and_ecs_counters():
    sx = _simulation()

    world = sx.World()
    empty = world.memory_diagnostics

    assert isinstance(empty, sx.WorldMemoryDiagnostics)
    assert isinstance(
        empty.allocator_debug_diagnostics, sx.MemoryManagerDebugDiagnostics
    )
    assert isinstance(
        empty.allocator_debug_diagnostics.free_allocator,
        sx.AllocatorDebugDiagnostics,
    )
    assert isinstance(empty.ecs_diagnostics, sx.WorldEcsDiagnostics)
    assert empty.frame_scratch_capacity_bytes > 0
    assert empty.frame_scratch_used_bytes == 0
    assert empty.ecs_diagnostics.entity_count == 0
    assert empty.ecs_diagnostics.component_count == 0

    world.add_free_frame("diagnostic_frame")
    diagnostics = world.memory_diagnostics
    ecs = diagnostics.ecs_diagnostics

    assert ecs.entity_count == 1
    assert ecs.entity_capacity >= ecs.entity_count
    assert ecs.storage_count == len(ecs.storages)
    assert ecs.storage_count > 0
    assert sum(storage.size for storage in ecs.storages) == ecs.component_count
    assert sum(storage.capacity for storage in ecs.storages) == ecs.component_capacity
    assert any(
        isinstance(storage, sx.WorldEcsStorageDiagnostics) for storage in ecs.storages
    )
    assert all(storage.capacity >= storage.size for storage in ecs.storages)


def test_simulation_stub_tracks_public_runtime_symbols():
    sx = _simulation()
    repo_root = Path(__file__).resolve().parents[4]
    stub = (repo_root / "python" / "stubs" / "dartpy" / "simulation.pyi").read_text(
        encoding="utf-8"
    )

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
        "ComputeExecutor",
        "SequentialExecutor",
        "ParallelExecutor",
        "ComputeNodeExecutionProfile",
        "ComputeExecutionProfile",
        "WorldStepProfile",
        "WorldStepStageProfile",
        "DeactivationOptions",
        "AllocatorDebugDiagnostics",
        "MemoryManagerDebugDiagnostics",
        "WorldEcsStorageDiagnostics",
        "WorldEcsDiagnostics",
        "WorldMemoryDiagnostics",
    )
    for symbol in public_symbols:
        assert hasattr(sx, symbol), symbol
        assert f'"{symbol}"' in stub
        assert f"class {symbol}" in stub

    assert hasattr(sx, "add_skeleton")
    assert '"add_skeleton"' in stub
    assert "def add_skeleton(" in stub
    assert not hasattr(sx, "add_world")
    assert '"add_world"' not in stub
    assert "def add_world(" not in stub

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
        "add_articulated_fixed_joint",
        "add_articulated_revolute_joint",
        "add_articulated_prismatic_joint",
        "add_articulated_spherical_joint",
        "get_articulated_joint",
        "get_articulated_joints",
        "has_articulated_joint",
        "num_articulated_joints",
        "add_rigid_body_revolute_joint",
        "add_rigid_body_prismatic_joint",
        "add_rigid_body_spherical_joint",
        "add_rigid_body_distance_spring",
        "has_rigid_body_distance_spring",
        "get_rigid_body_distance_spring_parameters",
        "set_rigid_body_distance_spring_parameters",
        "get_rigid_body_joint",
        "get_rigid_body_joints",
        "has_rigid_body_fixed_joint",
        "has_rigid_body_joint",
        "num_rigid_body_joints",
        "num_rigid_body_fixed_joints",
        "memory_diagnostics",
        "deactivation_options",
        "deactivation_enabled",
        "is_sleeping",
        "deactivation_group_index",
        "is_valid",
        "step_profiling_enabled",
        "last_step_profile",
        "worker_count",
        "inline_threshold",
        "graph_profiles",
        "max_parallelism",
        "average_parallelism",
        "accelerated_backend_enabled",
        "def get_stage(",
        "def get_node(",
        "def summary(",
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
        "def addRigidBodyRevoluteJoint(",
        "def addRigidBodyPrismaticJoint(",
        "def addRigidBodySphericalJoint(",
        "def addArticulatedFixedJoint(",
        "def addArticulatedRevoluteJoint(",
        "def addArticulatedPrismaticJoint(",
        "def addArticulatedSphericalJoint(",
        "def getArticulatedJoint(",
        "def hasArticulatedJoint(",
        "def getArticulatedJoints(",
        "def getArticulatedJointCount(",
        "def getRigidBodyJoint(",
        "def hasRigidBodyJoint(",
        "def getRigidBodyJoints(",
        "def getRigidBodyJointCount(",
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


def test_simulation_stub_places_rigid_surface_ccd_obstacle_on_rigid_body():
    sx = _simulation()
    repo_root = Path(__file__).resolve().parents[4]
    stub = (repo_root / "python" / "stubs" / "dartpy" / "simulation.pyi").read_text(
        encoding="utf-8"
    )

    assert hasattr(sx.RigidBody, "is_deformable_surface_ccd_obstacle")
    assert not hasattr(sx.Link, "is_deformable_surface_ccd_obstacle")
    assert "is_deformable_surface_ccd_obstacle" in _stub_class_block(stub, "RigidBody")
    assert "is_deformable_surface_ccd_obstacle" not in _stub_class_block(stub, "Link")

    assert hasattr(sx.RigidBody, "is_deformable_ground_barrier")
    assert not hasattr(sx.Link, "is_deformable_ground_barrier")
    assert "is_deformable_ground_barrier" in _stub_class_block(stub, "RigidBody")
    assert "is_deformable_ground_barrier" not in _stub_class_block(stub, "Link")


def test_simulation_deactivation_options_and_sleep_state():
    sx = _simulation()

    options = sx.DeactivationOptions(enabled=True, time_until_sleep=0.02)
    world = sx.World(
        time_step=0.01,
        gravity=(0.0, 0.0, 0.0),
        deactivation_options=options,
    )
    assert world.deactivation_enabled
    assert world.deactivation_options.enabled
    assert world.deactivation_options.time_until_sleep == pytest.approx(0.02)

    body = world.add_rigid_body("box")
    assert not body.is_sleeping
    assert body.deactivation_group_index == -1

    world.step(3)
    assert body.is_sleeping
    assert body.deactivation_group_index >= 0

    options.enabled = False
    world.deactivation_options = options
    assert not world.deactivation_enabled
    assert not body.is_sleeping
    assert body.deactivation_group_index == -1


def test_simulation_state_space_metadata_value_object():
    sx = _simulation()

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


def test_simulation_world_smoke():
    sx = _simulation()

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


def test_simulation_world_clear_invalidates_articulated_joint_handles_from_python():
    sx = _simulation()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    arm = world.add_multibody("clear_arm")
    base = arm.add_link("base")
    body = arm.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    hinge = world.add_articulated_revolute_joint(
        "clear_hinge", body, axis=(0.0, 0.0, 1.0)
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])

    world.enter_simulation_mode()
    world.step()
    assert world.frame == 1
    assert world.time == pytest.approx(world.time_step)
    assert world.num_articulated_joints == 1
    assert world.has_articulated_joint("clear_hinge")
    assert world.get_articulated_joint("clear_hinge").child_link == body

    rotation = np.asarray(body.rotation, dtype=float)
    assert math.atan2(rotation[1, 0], rotation[0, 0]) == pytest.approx(
        hinge_speed * world.time_step, abs=1e-6
    )

    world.clear()

    assert not world.is_simulation_mode
    assert not arm.is_valid
    assert not base.is_valid
    assert not body.is_valid
    assert not hinge.is_valid
    assert world.frame == 0
    assert world.time == pytest.approx(0.0)
    assert world.time_step == pytest.approx(0.001)
    assert world.num_multibodies == 0
    assert world.num_articulated_joints == 0
    assert not world.has_articulated_joint("clear_hinge")
    assert world.get_articulated_joint("clear_hinge") is None
    assert len(world.get_articulated_joints()) == 0

    world.gravity = (0.0, 0.0, 0.0)
    world.time_step = 0.005
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    rebuilt_arm = world.add_multibody("rebuilt_clear_arm")
    rebuilt_base = rebuilt_arm.add_link("base")
    rebuilt_body = rebuilt_arm.add_link(
        "body",
        parent=rebuilt_base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    slider = world.add_articulated_prismatic_joint(
        "clear_slider", rebuilt_body, axis=(1.0, 0.0, 0.0)
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1000.0], [1000.0])

    assert slider.is_valid
    assert world.num_articulated_joints == 1
    assert world.has_articulated_joint("clear_slider")
    assert world.get_articulated_joint("clear_slider").child_link == rebuilt_body

    world.enter_simulation_mode()
    world.step()

    position = np.asarray(rebuilt_body.translation, dtype=float).reshape(3)
    assert position[0] == pytest.approx(slider_speed * world.time_step, abs=1e-6)
    assert position[1] == pytest.approx(0.0, abs=1e-6)
    assert position[2] == pytest.approx(0.0, abs=1e-6)


def test_simulation_world_rigid_body_fixed_joint_projects_captured_pose():
    sx = _simulation()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    base = world.add_rigid_body("base")
    base.is_static = True
    link = world.add_rigid_body("link", position=(1.0, 0.0, 0.0))

    joint = world.add_rigid_body_fixed_joint("base_to_link", base, link)
    assert joint.name == "base_to_link"
    assert joint.type == sx.JointType.FIXED
    assert joint.num_dofs == 0
    assert joint.break_force == pytest.approx(0.0)
    assert not joint.is_broken
    joint.break_force = 12.5
    assert joint.break_force == pytest.approx(12.5)
    joint.reset_breakage()
    assert not joint.is_broken
    default_avbd_start_stiffness = joint.avbd_start_stiffness
    assert default_avbd_start_stiffness > 0.0
    assert math.isinf(joint.avbd_linear_stiffness)
    assert math.isinf(joint.avbd_angular_stiffness)
    joint.avbd_start_stiffness = 1.0
    joint.avbd_linear_stiffness = 1000.0
    joint.avbd_angular_stiffness = 100.0
    assert joint.avbd_start_stiffness == pytest.approx(1.0)
    assert joint.avbd_linear_stiffness == pytest.approx(1000.0)
    assert joint.avbd_angular_stiffness == pytest.approx(100.0)
    with pytest.raises(Exception, match="finite and non-negative"):
        joint.break_force = -1.0
    with pytest.raises(Exception, match="finite and non-negative"):
        joint.break_force = math.inf
    with pytest.raises(Exception, match="finite and non-negative"):
        joint.avbd_start_stiffness = -1.0
    with pytest.raises(Exception, match="non-negative or infinity"):
        joint.avbd_linear_stiffness = math.nan
    with pytest.raises(Exception, match="non-negative or infinity"):
        joint.avbd_angular_stiffness = -1.0
    joint.avbd_start_stiffness = default_avbd_start_stiffness
    joint.avbd_linear_stiffness = math.inf
    joint.avbd_angular_stiffness = math.inf
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


def test_simulation_world_rigid_body_one_dof_joints_project_supported_axes():
    sx = _simulation()

    hinge_world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    hinge_base = hinge_world.add_rigid_body("base")
    hinge_base.is_static = True
    hinge_link = hinge_world.add_rigid_body("link", position=(1.0, 0.0, 0.0))
    hinge = hinge_world.add_rigid_body_revolute_joint(
        "base_to_link_hinge", hinge_base, hinge_link, axis=(0.0, 0.0, 1.0)
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert np.asarray(hinge.axis, dtype=float)[2] == pytest.approx(1.0)
    assert hinge_world.has_rigid_body_joint("base_to_link_hinge")
    assert not hinge_world.has_rigid_body_fixed_joint("base_to_link_hinge")
    assert hinge_world.num_rigid_body_joints == 1
    assert hinge_world.num_rigid_body_fixed_joints == 0
    assert hinge_world.get_rigid_body_joint("base_to_link_hinge").type == (
        sx.JointType.REVOLUTE
    )
    assert hinge_world.get_rigid_body_joint("missing") is None
    assert len(hinge_world.get_rigid_body_joints()) == 1

    drifted_hinge = np.eye(4)
    drifted_hinge[:3, 3] = (1.25, 0.25, 0.0)
    hinge_link.transform = drifted_hinge
    hinge_world.enter_simulation_mode()
    hinge_world.step()
    assert float(hinge_link.translation[0]) == pytest.approx(1.0, abs=0.05)
    assert float(hinge_link.translation[1]) == pytest.approx(0.0, abs=0.05)

    slider_world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    slider_base = slider_world.add_rigid_body("base")
    slider_base.is_static = True
    slider = slider_world.add_rigid_body("slider", position=(0.0, 0.0, 1.0))
    joint = slider_world.add_rigid_body_prismatic_joint(
        "base_to_slider", slider_base, slider, axis=(0.0, 0.0, 1.0)
    )
    assert joint.type == sx.JointType.PRISMATIC
    assert joint.num_dofs == 1
    drifted_slider = np.eye(4)
    drifted_slider[:3, 3] = (0.25, 0.0, 1.5)
    slider.transform = drifted_slider
    slider_world.enter_simulation_mode()
    slider_world.step()
    assert float(slider.translation[0]) == pytest.approx(0.0, abs=0.05)
    assert float(slider.translation[2]) == pytest.approx(1.5, abs=0.05)


def test_simulation_world_rigid_body_prismatic_velocity_motor_steps_from_python():
    sx = _simulation()

    world = sx.World(time_step=0.01, gravity=(0.0, 0.0, 0.0))
    base = world.add_rigid_body("base")
    base.is_static = True
    slider = world.add_rigid_body("slider", position=(0.0, 0.0, 1.0))
    joint = world.add_rigid_body_prismatic_joint(
        "base_to_slider", base, slider, axis=(0.0, 0.0, 1.0)
    )
    joint.actuator_type = sx.ActuatorType.VELOCITY
    joint.command_velocity = [0.8]
    joint.set_effort_limits([-1000.0], [1000.0])

    world.enter_simulation_mode()
    start_z = float(slider.translation[2])
    for _ in range(4):
        world.step()

    assert float(slider.translation[2]) > start_z + 0.015
    assert float(slider.linear_velocity[2]) == pytest.approx(0.8, abs=0.25)

    joint.command_velocity = [-0.5]
    for _ in range(4):
        world.step()

    assert float(slider.linear_velocity[2]) == pytest.approx(-0.5, abs=0.25)


def test_simulation_world_rigid_body_spherical_joint_projects_anchor_only():
    sx = _simulation()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    base = world.add_rigid_body("base")
    base.is_static = True
    link = world.add_rigid_body("link", position=(1.0, 0.0, 0.0))

    joint = world.add_rigid_body_spherical_joint("base_to_link_socket", base, link)
    assert joint.type == sx.JointType.SPHERICAL
    assert joint.num_dofs == 3
    assert world.has_rigid_body_joint("base_to_link_socket")
    assert not world.has_rigid_body_fixed_joint("base_to_link_socket")
    assert world.num_rigid_body_joints == 1
    assert world.num_rigid_body_fixed_joints == 0
    assert world.get_rigid_body_joint("base_to_link_socket").type == (
        sx.JointType.SPHERICAL
    )

    drifted_socket = np.eye(4)
    angle = 0.35
    drifted_socket[:3, :3] = [
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ]
    drifted_socket[:3, 3] = (1.25, 0.25, 0.0)
    link.transform = drifted_socket

    world.enter_simulation_mode()
    world.step()

    assert float(link.translation[0]) == pytest.approx(1.0, abs=0.05)
    assert float(link.translation[1]) == pytest.approx(0.0, abs=0.05)
    rotation = np.asarray(link.rotation, dtype=float)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    assert yaw == pytest.approx(angle, abs=0.02)

    anchored_world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    anchored_base = anchored_world.add_rigid_body("base")
    anchored_base.is_static = True
    anchored_link = anchored_world.add_rigid_body("link", position=(1.0, 0.0, 0.0))
    anchored_world.add_rigid_body_spherical_joint(
        "endpoint_socket",
        anchored_base,
        anchored_link,
        parent_anchor=(0.5, 0.0, 0.0),
        child_anchor=(-0.5, 0.0, 0.0),
    )
    drifted_endpoint = np.eye(4)
    drifted_endpoint[:3, 3] = (1.25, 0.25, 0.0)
    anchored_link.transform = drifted_endpoint

    anchored_world.enter_simulation_mode()
    anchored_world.step()

    base_endpoint = np.asarray(anchored_base.translation, dtype=float).reshape(
        3
    ) + np.asarray(anchored_base.rotation, dtype=float) @ np.array([0.5, 0.0, 0.0])
    child_endpoint = np.asarray(anchored_link.translation, dtype=float).reshape(
        3
    ) + np.asarray(anchored_link.rotation, dtype=float) @ np.array([-0.5, 0.0, 0.0])
    assert np.linalg.norm(base_endpoint - child_endpoint) < 0.08


def test_simulation_world_rigid_body_distance_spring_reduces_stretch():
    sx = _simulation()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    base = world.add_rigid_body("base")
    base.is_static = True
    link = world.add_rigid_body("link", position=(2.0, 0.0, 0.0))

    with pytest.raises(Exception, match="rest length"):
        world.add_rigid_body_distance_spring("bad_rest", base, link, -1.0, 100.0)
    with pytest.raises(Exception, match="stiffness"):
        world.add_rigid_body_distance_spring("bad_stiffness", base, link, 1.0, 0.0)

    assert (
        world.add_rigid_body_distance_spring("radial_spring", base, link, 1.0, 200.0)
        is None
    )
    assert world.has_rigid_body_distance_spring("radial_spring")
    assert not world.has_rigid_body_distance_spring("missing_spring")
    assert world.get_rigid_body_distance_spring_parameters(
        "radial_spring"
    ) == pytest.approx((1.0, 200.0))
    with pytest.raises(Exception, match="does not exist"):
        world.get_rigid_body_distance_spring_parameters("missing_spring")
    with pytest.raises(Exception, match="does not exist"):
        world.set_rigid_body_distance_spring_parameters("missing_spring", 1.0, 200.0)
    with pytest.raises(Exception, match="rest length"):
        world.set_rigid_body_distance_spring_parameters("radial_spring", -1.0, 200.0)
    with pytest.raises(Exception, match="stiffness"):
        world.set_rigid_body_distance_spring_parameters("radial_spring", 1.0, 0.0)
    world.set_rigid_body_distance_spring_parameters("radial_spring", 0.75, 80.0)
    assert world.get_rigid_body_distance_spring_parameters(
        "radial_spring"
    ) == pytest.approx((0.75, 80.0))
    with pytest.raises(Exception, match="already exists"):
        world.add_rigid_body_distance_spring("radial_spring", base, link, 1.0, 200.0)

    initial_distance = np.linalg.norm(
        np.asarray(link.translation, dtype=float)
        - np.asarray(base.translation, dtype=float)
    )
    world.enter_simulation_mode()
    world.step()
    projected_distance = np.linalg.norm(
        np.asarray(link.translation, dtype=float)
        - np.asarray(base.translation, dtype=float)
    )

    assert projected_distance < initial_distance
    assert float(link.linear_velocity[0]) < 0.0
    world.set_rigid_body_distance_spring_parameters("radial_spring", 1.25, 120.0)
    assert world.get_rigid_body_distance_spring_parameters(
        "radial_spring"
    ) == pytest.approx((1.25, 120.0))
    with pytest.raises(Exception, match="simulation mode"):
        world.add_rigid_body_distance_spring("late", base, link, 1.0, 200.0)


def test_simulation_world_rigid_body_fixed_joint_list_keeps_world_alive():
    sx = _simulation()

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


def test_simulation_world_articulated_joint_list_keeps_world_alive():
    sx = _simulation()

    def build_joints_from_temporary_world():
        world = sx.World()
        world.multibody_options = sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
        )
        arm = world.add_multibody("arm")
        base = arm.add_link("base")
        body = arm.add_link(
            "body",
            parent=base,
            joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
        )
        world.add_articulated_fixed_joint("base_hold", base, body)
        world.add_articulated_revolute_joint(
            "world_hinge", body, axis=(0.0, 1.0, 0.0)
        )
        return world.get_articulated_joints()

    joints = build_joints_from_temporary_world()
    gc.collect()

    joints_by_name = {joint.name: joint for joint in joints}
    assert set(joints_by_name) == {"base_hold", "world_hinge"}
    assert joints_by_name["base_hold"].parent_link.name == "base"
    assert joints_by_name["base_hold"].child_link.name == "body"
    with pytest.raises(Exception, match="parent endpoint"):
        _ = joints_by_name["world_hinge"].parent_link
    assert joints_by_name["world_hinge"].child_link.name == "body"


def test_simulation_world_articulated_point_joint_facade_exposes_link_endpoints():
    sx = _simulation()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody("arm")
    base = arm.add_link("base")
    body = arm.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )

    fixed = world.add_articulated_fixed_joint("base_hold", base, body)
    assert fixed.type == sx.JointType.FIXED
    assert fixed.num_dofs == 0
    assert fixed.parent_link == base
    assert fixed.child_link == body
    assert math.isinf(fixed.avbd_start_stiffness)
    assert math.isinf(fixed.avbd_linear_stiffness)
    assert math.isinf(fixed.avbd_angular_stiffness)
    fixed.avbd_start_stiffness = 2.0
    fixed.avbd_linear_stiffness = 200.0
    fixed.avbd_angular_stiffness = 300.0
    assert fixed.avbd_start_stiffness == pytest.approx(2.0)
    assert fixed.avbd_linear_stiffness == pytest.approx(200.0)
    assert fixed.avbd_angular_stiffness == pytest.approx(300.0)
    assert world.has_articulated_joint("base_hold")
    assert world.get_articulated_joint("base_hold").child_link == body
    assert world.get_articulated_joint("missing") is None
    assert world.get_rigid_body_joint("base_hold") is None

    hinge = world.add_articulated_revolute_joint(
        "base_hinge", base, body, axis=(0.0, 0.0, 2.0)
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert np.asarray(hinge.axis, dtype=float)[2] == pytest.approx(1.0)
    assert math.isinf(hinge.avbd_start_stiffness)
    hinge.avbd_start_stiffness = 3.0
    hinge.avbd_linear_stiffness = 400.0
    hinge.avbd_angular_stiffness = 500.0
    assert hinge.avbd_start_stiffness == pytest.approx(3.0)
    assert hinge.avbd_linear_stiffness == pytest.approx(400.0)
    assert hinge.avbd_angular_stiffness == pytest.approx(500.0)

    slider = world.add_articulated_prismatic_joint(
        "base_slider", base, body, axis=(1.0, 0.0, 0.0)
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert math.isinf(slider.avbd_start_stiffness)
    slider.avbd_start_stiffness = 4.0
    slider.avbd_linear_stiffness = 600.0
    slider.avbd_angular_stiffness = 700.0
    assert slider.avbd_start_stiffness == pytest.approx(4.0)
    assert slider.avbd_linear_stiffness == pytest.approx(600.0)
    assert slider.avbd_angular_stiffness == pytest.approx(700.0)

    socket = world.add_articulated_spherical_joint("base_socket", base, body)
    assert socket.type == sx.JointType.SPHERICAL
    assert socket.num_dofs == 3
    assert math.isinf(socket.avbd_start_stiffness)
    socket.avbd_start_stiffness = 5.0
    socket.avbd_linear_stiffness = 800.0
    socket.avbd_angular_stiffness = 900.0
    assert socket.avbd_start_stiffness == pytest.approx(5.0)
    assert socket.avbd_linear_stiffness == pytest.approx(800.0)
    assert socket.avbd_angular_stiffness == pytest.approx(900.0)

    offset_socket = world.add_articulated_spherical_joint(
        "offset_socket",
        base,
        body,
        parent_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert offset_socket.type == sx.JointType.SPHERICAL
    assert offset_socket.num_dofs == 3

    offset_hinge = world.add_articulated_revolute_joint(
        "offset_hinge",
        base,
        body,
        axis=(0.0, 0.0, 2.0),
        parent_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert offset_hinge.type == sx.JointType.REVOLUTE

    offset_slider = world.add_articulated_prismatic_joint(
        "offset_slider",
        base,
        body,
        axis=(1.0, 0.0, 0.0),
        parent_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert offset_slider.type == sx.JointType.PRISMATIC

    world_hold = world.add_articulated_fixed_joint("world_hold", body)
    assert world_hold.type == sx.JointType.FIXED
    assert world_hold.child_link == body
    assert math.isinf(world_hold.avbd_start_stiffness)
    world_hold.avbd_start_stiffness = 6.0
    world_hold.avbd_linear_stiffness = 1000.0
    world_hold.avbd_angular_stiffness = 1100.0
    assert world_hold.avbd_start_stiffness == pytest.approx(6.0)
    assert world_hold.avbd_linear_stiffness == pytest.approx(1000.0)
    assert world_hold.avbd_angular_stiffness == pytest.approx(1100.0)
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_hold.parent_link

    world_offset = world.add_articulated_fixed_joint(
        "world_offset",
        body,
        world_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert world_offset.type == sx.JointType.FIXED

    world_hinge = world.add_articulated_revolute_joint(
        "world_hinge", body, axis=(0.0, 0.0, 2.0)
    )
    assert world_hinge.type == sx.JointType.REVOLUTE
    assert world_hinge.num_dofs == 1
    assert np.asarray(world_hinge.axis, dtype=float)[2] == pytest.approx(1.0)

    world_slider = world.add_articulated_prismatic_joint(
        "world_slider", body, axis=(0.0, 1.0, 0.0)
    )
    assert world_slider.type == sx.JointType.PRISMATIC
    assert world_slider.num_dofs == 1

    world_offset_hinge = world.add_articulated_revolute_joint(
        "world_offset_hinge",
        body,
        axis=(0.0, 0.0, 2.0),
        world_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert world_offset_hinge.type == sx.JointType.REVOLUTE

    world_offset_slider = world.add_articulated_prismatic_joint(
        "world_offset_slider",
        body,
        axis=(0.0, 1.0, 0.0),
        world_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert world_offset_slider.type == sx.JointType.PRISMATIC

    world_socket = world.add_articulated_spherical_joint("world_socket", body)
    assert world_socket.type == sx.JointType.SPHERICAL
    assert world_socket.num_dofs == 3
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_socket.parent_link

    world_offset_socket = world.add_articulated_spherical_joint(
        "world_offset_socket",
        body,
        world_anchor=(0.2, 0.0, 0.0),
        child_anchor=(-0.1, 0.0, 0.0),
    )
    assert world_offset_socket.type == sx.JointType.SPHERICAL
    assert world_offset_socket.num_dofs == 3

    assert world.num_articulated_joints == 15
    assert len(world.get_articulated_joints()) == 15

    other_arm = world.add_multibody("other_arm")
    other_base = other_arm.add_link("other_base")
    other_body = other_arm.add_link(
        "other_body",
        parent=other_base,
        joint=sx.JointSpec(name="other_floating", type=sx.JointType.FLOATING),
    )
    foreign_world = sx.World()
    foreign_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    foreign_arm = foreign_world.add_multibody("foreign_arm")
    foreign_base = foreign_arm.add_link("foreign_base")

    with pytest.raises(Exception, match="already exists"):
        world.add_articulated_fixed_joint("base_hold", base, body)
    with pytest.raises(Exception, match="distinct"):
        world.add_articulated_fixed_joint("same_link", base, base)
    with pytest.raises(Exception, match="same multibody"):
        world.add_articulated_revolute_joint(
            "cross_multibody", base, other_body, axis=(0.0, 0.0, 1.0)
        )
    with pytest.raises(Exception, match="this World"):
        world.add_articulated_prismatic_joint(
            "cross_world", base, foreign_base, axis=(1.0, 0.0, 0.0)
        )
    with pytest.raises(Exception, match="this World"):
        world.add_articulated_spherical_joint("world_cross_world", foreign_base)
    with pytest.raises(Exception, match="finite and non-zero"):
        world.add_articulated_revolute_joint(
            "bad_axis", base, body, axis=(0.0, 0.0, 0.0)
        )
    with pytest.raises(Exception, match="both endpoints"):
        world.add_articulated_fixed_joint(
            "missing_anchor",
            base,
            body,
            parent_anchor=(0.0, 0.0, 0.0),
        )
    with pytest.raises(Exception, match="anchors must be finite"):
        world.add_articulated_fixed_joint(
            "bad_anchor",
            base,
            body,
            parent_anchor=(math.nan, 0.0, 0.0),
            child_anchor=(0.0, 0.0, 0.0),
        )
    with pytest.raises(Exception, match="finite and non-negative"):
        fixed.avbd_start_stiffness = math.inf
    with pytest.raises(Exception, match="non-negative or infinity"):
        hinge.avbd_linear_stiffness = math.nan
    with pytest.raises(Exception, match="non-negative or infinity"):
        slider.avbd_angular_stiffness = -1.0

    world.enter_simulation_mode()
    with pytest.raises(Exception, match="simulation mode"):
        world.add_articulated_fixed_joint("late_joint", base, body)


def test_simulation_world_articulated_point_joints_generate_unique_names():
    sx = _simulation()

    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody("generated_joint_arm")
    base = arm.add_link("base")
    body = arm.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )

    explicit_joint = world.add_articulated_fixed_joint("joint_001", base, body)
    generated_fixed = world.add_articulated_fixed_joint("", base, body)
    generated_world_hinge = world.add_articulated_revolute_joint(
        "", body, axis=(0.0, 1.0, 0.0)
    )
    generated_slider = world.add_articulated_prismatic_joint(
        "", base, body, axis=(1.0, 0.0, 0.0)
    )

    assert explicit_joint.name == "joint_001"
    assert generated_fixed.name == "joint_002"
    assert generated_world_hinge.name == "joint_003"
    assert generated_slider.name == "joint_004"
    assert world.num_articulated_joints == 4

    joints_by_name = {
        joint.name: joint.type for joint in world.get_articulated_joints()
    }
    assert joints_by_name == {
        "joint_001": sx.JointType.FIXED,
        "joint_002": sx.JointType.FIXED,
        "joint_003": sx.JointType.REVOLUTE,
        "joint_004": sx.JointType.PRISMATIC,
    }
    assert world.get_articulated_joint("joint_002").child_link == body
    assert world.has_articulated_joint("joint_003")
    assert world.get_articulated_joint("joint_005") is None
    with pytest.raises(Exception, match="already exists"):
        world.add_articulated_spherical_joint("joint_004", base, body)


def test_simulation_world_articulated_generated_names_resume_after_binary_roundtrip(
    tmp_path: Path,
):
    sx = _simulation()

    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody("serialized_generated_joint_arm")
    base = arm.add_link("base")
    body = arm.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="tree", type=sx.JointType.FLOATING),
    )

    explicit_joint = world.add_articulated_fixed_joint("joint_001", base, body)
    generated_joint = world.add_articulated_revolute_joint(
        "", body, axis=(0.0, 1.0, 0.0)
    )
    assert explicit_joint.name == "joint_001"
    assert generated_joint.name == "joint_002"

    binary_path = tmp_path / "articulated_generated_names.bin"
    world.save_binary(binary_path)

    restored_world = sx.World()
    restored_world.load_binary(binary_path)
    restored_arm = restored_world.get_multibody("serialized_generated_joint_arm")
    assert restored_arm is not None
    restored_base = restored_arm.get_link("base")
    restored_body = restored_arm.get_link("body")
    assert restored_base is not None
    assert restored_body is not None
    assert restored_world.has_articulated_joint("joint_001")
    assert restored_world.has_articulated_joint("joint_002")
    assert restored_world.num_articulated_joints == 2

    generated_after_load = restored_world.add_articulated_prismatic_joint(
        "", restored_base, restored_body, axis=(1.0, 0.0, 0.0)
    )
    assert generated_after_load.name == "joint_004"
    assert restored_world.num_articulated_joints == 3


def test_simulation_world_articulated_avbd_stiffness_roundtrip_from_python(
    tmp_path: Path,
):
    sx = _simulation()

    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody("serialized_articulated_stiffness_arm")
    base = arm.add_link("base")

    def add_body(name: str):
        return arm.add_link(
            name,
            parent=base,
            joint=sx.JointSpec(
                name=f"{name}_float", type=sx.JointType.FLOATING
            ),
        )

    fixed_body = add_body("fixed_body")
    hinge_body = add_body("hinge_body")
    slider_body = add_body("slider_body")
    socket_body = add_body("socket_body")
    world_fixed_body = add_body("world_fixed_body")
    world_hinge_body = add_body("world_hinge_body")
    world_slider_body = add_body("world_slider_body")
    world_socket_body = add_body("world_socket_body")

    fixed = world.add_articulated_fixed_joint("stiff_fixed", base, fixed_body)
    fixed.avbd_start_stiffness = 3.0
    fixed.avbd_linear_stiffness = 234.0
    fixed.avbd_angular_stiffness = 567.0

    hinge = world.add_articulated_revolute_joint(
        "stiff_hinge", base, hinge_body, axis=(0.0, 1.0, 0.0)
    )
    hinge.avbd_start_stiffness = 4.0
    hinge.avbd_linear_stiffness = 345.0
    hinge.avbd_angular_stiffness = 678.0

    slider = world.add_articulated_prismatic_joint(
        "stiff_slider", base, slider_body, axis=(1.0, 0.0, 0.0)
    )
    slider.avbd_start_stiffness = 5.0
    slider.avbd_linear_stiffness = 456.0
    slider.avbd_angular_stiffness = 789.0

    socket = world.add_articulated_spherical_joint(
        "stiff_socket", base, socket_body
    )
    socket.avbd_start_stiffness = 6.0
    socket.avbd_linear_stiffness = 567.0
    socket.avbd_angular_stiffness = 890.0

    world_fixed = world.add_articulated_fixed_joint(
        "stiff_world_fixed", world_fixed_body
    )
    world_fixed.avbd_start_stiffness = 7.0
    world_fixed.avbd_linear_stiffness = 678.0
    world_fixed.avbd_angular_stiffness = 901.0

    world_hinge = world.add_articulated_revolute_joint(
        "stiff_world_hinge", world_hinge_body, axis=(0.0, 1.0, 0.0)
    )
    world_hinge.avbd_start_stiffness = 8.0
    world_hinge.avbd_linear_stiffness = 789.0
    world_hinge.avbd_angular_stiffness = 1012.0

    world_slider = world.add_articulated_prismatic_joint(
        "stiff_world_slider", world_slider_body, axis=(1.0, 0.0, 0.0)
    )
    world_slider.avbd_start_stiffness = 9.0
    world_slider.avbd_linear_stiffness = 890.0
    world_slider.avbd_angular_stiffness = 1123.0

    world_socket = world.add_articulated_spherical_joint(
        "stiff_world_socket", world_socket_body
    )
    world_socket.avbd_start_stiffness = 10.0
    world_socket.avbd_linear_stiffness = 901.0
    world_socket.avbd_angular_stiffness = 1234.0

    binary_path = tmp_path / "articulated_avbd_stiffness.bin"
    world.save_binary(binary_path)

    restored_world = sx.World()
    restored_world.load_binary(binary_path)
    expected = {
        "stiff_fixed": (sx.JointType.FIXED, 0, 3.0, 234.0, 567.0),
        "stiff_hinge": (sx.JointType.REVOLUTE, 1, 4.0, 345.0, 678.0),
        "stiff_slider": (sx.JointType.PRISMATIC, 1, 5.0, 456.0, 789.0),
        "stiff_socket": (sx.JointType.SPHERICAL, 3, 6.0, 567.0, 890.0),
        "stiff_world_fixed": (sx.JointType.FIXED, 0, 7.0, 678.0, 901.0),
        "stiff_world_hinge": (sx.JointType.REVOLUTE, 1, 8.0, 789.0, 1012.0),
        "stiff_world_slider": (
            sx.JointType.PRISMATIC,
            1,
            9.0,
            890.0,
            1123.0,
        ),
        "stiff_world_socket": (sx.JointType.SPHERICAL, 3, 10.0, 901.0, 1234.0),
    }
    for name, (joint_type, num_dofs, start, linear, angular) in expected.items():
        restored = restored_world.get_articulated_joint(name)
        assert restored is not None
        assert restored.type == joint_type
        assert restored.num_dofs == num_dofs
        assert restored.avbd_start_stiffness == pytest.approx(start)
        assert restored.avbd_linear_stiffness == pytest.approx(linear)
        assert restored.avbd_angular_stiffness == pytest.approx(angular)

    restored_updates = {
        "stiff_fixed": (432.0, 765.0),
        "stiff_hinge": (543.0, 876.0),
        "stiff_slider": (654.0, 987.0),
        "stiff_socket": (765.0, 1098.0),
        "stiff_world_fixed": (876.0, 1209.0),
        "stiff_world_hinge": (987.0, 1320.0),
        "stiff_world_slider": (1098.0, 1431.0),
        "stiff_world_socket": (1209.0, 1542.0),
    }
    for name, (linear, angular) in restored_updates.items():
        restored = restored_world.get_articulated_joint(name)
        assert restored is not None
        restored.avbd_linear_stiffness = linear
        restored.avbd_angular_stiffness = angular

    restored_world.enter_simulation_mode()
    for name, (_, _, start, _, _) in expected.items():
        linear, angular = restored_updates[name]
        restored = restored_world.get_articulated_joint(name)
        assert restored is not None
        assert restored.avbd_start_stiffness == pytest.approx(start)
        assert restored.avbd_linear_stiffness == pytest.approx(linear)
        assert restored.avbd_angular_stiffness == pytest.approx(angular)


def test_simulation_world_clear_resets_articulated_generated_names():
    sx = _simulation()

    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    arm = world.add_multibody("clear_generated_joint_arm")
    base = arm.add_link("base")
    body = arm.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )

    first_generated = world.add_articulated_fixed_joint("", base, body)
    assert first_generated.name == "joint_001"
    assert world.num_articulated_joints == 1
    assert base.is_valid
    assert body.is_valid
    assert first_generated.is_valid

    world.clear()
    assert world.num_articulated_joints == 0
    assert not world.has_articulated_joint("joint_001")
    assert not base.is_valid
    assert not body.is_valid
    assert not first_generated.is_valid

    rebuilt = world.add_multibody("clear_generated_joint_rebuilt")
    rebuilt_base = rebuilt.add_link("base")
    rebuilt_body = rebuilt.add_link(
        "body",
        parent=rebuilt_base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )

    regenerated = world.add_articulated_revolute_joint(
        "", rebuilt_body, axis=(0.0, 0.0, 1.0)
    )
    assert regenerated.name == "joint_001"
    assert regenerated.type == sx.JointType.REVOLUTE
    assert world.num_articulated_joints == 1


def test_simulation_world_articulated_motor_breakage_steps_from_python():
    sx = _simulation()

    hinge_world, hinge_base, hinge_body = _floating_link_world(
        sx, "python_breakable_hinge"
    )
    hinge_body.parent_joint.position = [0.1, -0.2, 0.15, 0.0, 0.0, 0.0]
    hinge_child_anchor = np.array([-0.15, 0.05, 0.0])
    hinge_parent_anchor = (
        np.asarray(hinge_body.translation, dtype=float)
        + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
    )
    hinge = hinge_world.add_articulated_revolute_joint(
        "breakable_hinge",
        hinge_base,
        hinge_body,
        axis=(0.0, 0.0, 1.0),
        parent_anchor=hinge_parent_anchor.tolist(),
        child_anchor=hinge_child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == hinge_base
    assert hinge.child_link == hinge_body
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    target_speed = 0.4
    hinge.command_velocity = [target_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    hinge_world.enter_simulation_mode()
    hinge_world.step()

    assert hinge.is_broken
    rotation = np.asarray(hinge_body.rotation, dtype=float)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    assert yaw == pytest.approx(target_speed * hinge_world.time_step, abs=1e-6)

    max_anchor_residual = 0.0
    for _ in range(20):
        hinge_body.apply_force((0.0, 4.0, 0.0))
        hinge_world.step()
        anchor_position = (
            np.asarray(hinge_body.translation, dtype=float)
            + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
        )
        max_anchor_residual = max(
            max_anchor_residual,
            float(np.linalg.norm(anchor_position - hinge_parent_anchor)),
        )
    assert max_anchor_residual > 1e-4

    slider_world, _, slider_body = _floating_link_world(
        sx, "python_world_breakable_slider"
    )
    slider_body.parent_joint.position = [0.2, -0.15, 0.05, 0.0, 0.0, 0.0]
    slider_axis = np.array([1.0, 0.0, 0.0])
    slider_child_anchor = np.array([0.0, 0.2, 0.1])
    slider_world_anchor = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider = slider_world.add_articulated_prismatic_joint(
        "world_breakable_slider",
        slider_body,
        axis=slider_axis,
        world_anchor=slider_world_anchor.tolist(),
        child_anchor=slider_child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.child_link == slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    target_speed = 0.3
    slider.command_velocity = [target_speed]
    slider.set_effort_limits([-1000.0], [1000.0])
    slider.break_force = 1e-18

    slider_world.enter_simulation_mode()
    slider_world.step()

    assert slider.is_broken
    anchor_position = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    anchor_delta = anchor_position - slider_world_anchor
    assert float(anchor_delta.dot(slider_axis)) == pytest.approx(
        target_speed * slider_world.time_step, abs=1e-6
    )
    orthogonal_delta = anchor_delta - anchor_delta.dot(slider_axis) * slider_axis
    assert np.linalg.norm(orthogonal_delta) < 1e-6

    max_orthogonal_drift = 0.0
    for _ in range(20):
        slider_body.apply_force((0.0, 4.0, 0.0))
        slider_world.step()
        anchor_position = (
            np.asarray(slider_body.translation, dtype=float)
            + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
        )
        anchor_delta = anchor_position - slider_world_anchor
        orthogonal = anchor_delta - anchor_delta.dot(slider_axis) * slider_axis
        max_orthogonal_drift = max(
            max_orthogonal_drift, float(np.linalg.norm(orthogonal))
        )
    assert max_orthogonal_drift > 1e-4


def test_simulation_world_articulated_motor_breakage_reset_reengages_from_python():
    sx = _simulation()

    hinge_world, hinge_base, hinge_body = _floating_link_world(
        sx, "python_resettable_breakable_hinge"
    )
    hinge_body.parent_joint.position = [0.1, -0.2, 0.15, 0.0, 0.0, 0.0]
    hinge_child_anchor = np.array([-0.15, 0.05, 0.0])
    hinge_parent_anchor = (
        np.asarray(hinge_body.translation, dtype=float)
        + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
    )
    hinge = hinge_world.add_articulated_revolute_joint(
        "resettable_hinge",
        hinge_base,
        hinge_body,
        axis=(0.0, 0.0, 1.0),
        parent_anchor=hinge_parent_anchor.tolist(),
        child_anchor=hinge_child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == hinge_base
    assert hinge.child_link == hinge_body
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    hinge_world.enter_simulation_mode()
    hinge_world.step()

    assert hinge.is_broken
    hinge_yaw = math.atan2(
        np.asarray(hinge_body.rotation, dtype=float)[1, 0],
        np.asarray(hinge_body.rotation, dtype=float)[0, 0],
    )
    assert hinge_yaw == pytest.approx(hinge_speed * hinge_world.time_step, abs=1e-6)

    max_hinge_anchor_residual = 0.0
    for _ in range(20):
        hinge_body.apply_force((0.0, 4.0, 0.0))
        hinge_world.step()
        hinge_anchor_position = (
            np.asarray(hinge_body.translation, dtype=float)
            + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
        )
        max_hinge_anchor_residual = max(
            max_hinge_anchor_residual,
            float(np.linalg.norm(hinge_anchor_position - hinge_parent_anchor)),
        )
    assert max_hinge_anchor_residual > 1e-4
    hinge_pre_reset_yaw = math.atan2(
        np.asarray(hinge_body.rotation, dtype=float)[1, 0],
        np.asarray(hinge_body.rotation, dtype=float)[0, 0],
    )

    hinge_body.parent_joint.velocity = [0.0] * 6
    hinge.break_force = 1.0e12
    hinge.reset_breakage()
    assert not hinge.is_broken

    hinge_world.step()

    assert not hinge.is_broken
    assert hinge.parent_link == hinge_base
    assert hinge.child_link == hinge_body
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    hinge_reset_anchor_position = (
        np.asarray(hinge_body.translation, dtype=float)
        + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
    )
    assert np.linalg.norm(hinge_reset_anchor_position - hinge_parent_anchor) < 1e-6
    assert (
        np.linalg.norm(
            np.asarray(hinge_body.rotation, dtype=float) @ np.array([0.0, 0.0, 1.0])
            - np.array([0.0, 0.0, 1.0])
        )
        < 1e-6
    )
    hinge_reset_yaw = math.atan2(
        np.asarray(hinge_body.rotation, dtype=float)[1, 0],
        np.asarray(hinge_body.rotation, dtype=float)[0, 0],
    )
    assert hinge_reset_yaw > hinge_pre_reset_yaw

    slider_world, _, slider_body = _floating_link_world(
        sx, "python_resettable_world_breakable_slider"
    )
    slider_body.parent_joint.position = [0.2, -0.15, 0.05, 0.0, 0.0, 0.0]
    slider_axis = np.array([1.0, 0.0, 0.0])
    slider_child_anchor = np.array([0.0, 0.2, 0.1])
    slider_world_anchor = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider = slider_world.add_articulated_prismatic_joint(
        "resettable_world_slider",
        slider_body,
        axis=slider_axis,
        world_anchor=slider_world_anchor.tolist(),
        child_anchor=slider_child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.child_link == slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1000.0], [1000.0])
    slider.break_force = 1e-18

    slider_world.enter_simulation_mode()
    slider_world.step()

    assert slider.is_broken
    slider_anchor_position = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider_anchor_delta = slider_anchor_position - slider_world_anchor
    assert float(slider_anchor_delta.dot(slider_axis)) == pytest.approx(
        slider_speed * slider_world.time_step, abs=1e-6
    )
    slider_orthogonal_delta = (
        slider_anchor_delta - slider_anchor_delta.dot(slider_axis) * slider_axis
    )
    assert np.linalg.norm(slider_orthogonal_delta) < 1e-6

    max_slider_orthogonal_drift = 0.0
    for _ in range(20):
        slider_body.apply_force((0.0, 4.0, 0.0))
        slider_world.step()
        slider_anchor_position = (
            np.asarray(slider_body.translation, dtype=float)
            + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
        )
        slider_force_delta = slider_anchor_position - slider_world_anchor
        slider_force_orthogonal = (
            slider_force_delta - slider_force_delta.dot(slider_axis) * slider_axis
        )
        max_slider_orthogonal_drift = max(
            max_slider_orthogonal_drift,
            float(np.linalg.norm(slider_force_orthogonal)),
        )
    assert max_slider_orthogonal_drift > 1e-4

    slider_body.parent_joint.velocity = [0.0] * 6
    slider.break_force = 1.0e12
    slider.reset_breakage()
    assert not slider.is_broken

    slider_world.step()

    assert not slider.is_broken
    assert slider.child_link == slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider_reset_anchor_position = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider_reset_delta = slider_reset_anchor_position - slider_world_anchor
    slider_reset_orthogonal = (
        slider_reset_delta - slider_reset_delta.dot(slider_axis) * slider_axis
    )
    assert np.linalg.norm(slider_reset_orthogonal) < 1e-6
    assert (
        np.linalg.norm(
            np.asarray(slider_body.rotation, dtype=float) - np.eye(3)
        )
        < 1e-6
    )
    assert float(slider_reset_delta.dot(slider_axis)) > float(
        slider_anchor_delta.dot(slider_axis)
    )


def test_simulation_world_articulated_complementary_motor_breakage_reset_from_python():
    sx = _simulation()

    slider_world, slider_base, slider_body = _floating_link_world(
        sx, "python_resettable_same_breakable_slider"
    )
    slider_body.parent_joint.position = [0.05, -0.1, 0.0, 0.0, 0.0, 0.0]
    slider_axis = np.array([1.0, 0.0, 0.0])
    slider_child_anchor = np.array([0.0, 0.2, 0.1])
    slider_parent_anchor = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider = slider_world.add_articulated_prismatic_joint(
        "resettable_same_slider",
        slider_base,
        slider_body,
        axis=slider_axis,
        parent_anchor=slider_parent_anchor.tolist(),
        child_anchor=slider_child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.parent_link == slider_base
    assert slider.child_link == slider_body
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1000.0], [1000.0])
    slider.break_force = 1e-18

    slider_world.enter_simulation_mode()
    slider_world.step()

    assert slider.is_broken
    slider_anchor_position = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider_anchor_delta = slider_anchor_position - slider_parent_anchor
    assert float(slider_anchor_delta.dot(slider_axis)) == pytest.approx(
        slider_speed * slider_world.time_step, abs=1e-6
    )

    max_orthogonal_drift = 0.0
    for _ in range(20):
        slider_body.apply_force((0.0, 4.0, 0.0))
        slider_world.step()
        anchor_position = (
            np.asarray(slider_body.translation, dtype=float)
            + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
        )
        anchor_delta = anchor_position - slider_parent_anchor
        orthogonal = anchor_delta - anchor_delta.dot(slider_axis) * slider_axis
        max_orthogonal_drift = max(
            max_orthogonal_drift, float(np.linalg.norm(orthogonal))
        )
    assert max_orthogonal_drift > 1e-4

    slider_body.parent_joint.velocity = [0.0] * 6
    slider.break_force = 1.0e12
    slider.reset_breakage()

    assert not slider.is_broken

    slider_world.step()

    assert not slider.is_broken
    assert slider.parent_link == slider_base
    assert slider.child_link == slider_body
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider_reset_anchor_position = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider_reset_delta = slider_reset_anchor_position - slider_parent_anchor
    slider_reset_orthogonal = (
        slider_reset_delta - slider_reset_delta.dot(slider_axis) * slider_axis
    )
    assert np.linalg.norm(slider_reset_orthogonal) < 1e-6
    assert (
        np.linalg.norm(
            np.asarray(slider_body.rotation, dtype=float) - np.eye(3)
        )
        < 1e-6
    )
    assert float(slider_reset_delta.dot(slider_axis)) > float(
        slider_anchor_delta.dot(slider_axis)
    )

    hinge_world, _, hinge_body = _floating_link_world(
        sx, "python_resettable_world_breakable_hinge"
    )
    hinge_body.parent_joint.position = [0.15, -0.05, 0.0, 0.0, 0.0, 0.0]
    hinge_child_anchor = np.array([0.2, 0.1, 0.0])
    hinge_world_anchor = (
        np.asarray(hinge_body.translation, dtype=float)
        + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
    )
    hinge = hinge_world.add_articulated_revolute_joint(
        "resettable_world_hinge",
        hinge_body,
        axis=(0.0, 0.0, 1.0),
        world_anchor=hinge_world_anchor.tolist(),
        child_anchor=hinge_child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.child_link == hinge_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = hinge.parent_link
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    hinge_world.enter_simulation_mode()
    hinge_world.step()

    assert hinge.is_broken
    hinge_yaw = math.atan2(
        np.asarray(hinge_body.rotation, dtype=float)[1, 0],
        np.asarray(hinge_body.rotation, dtype=float)[0, 0],
    )
    assert hinge_yaw == pytest.approx(hinge_speed * hinge_world.time_step, abs=1e-6)

    max_anchor_residual = 0.0
    for _ in range(20):
        hinge_body.apply_force((0.0, 4.0, 0.0))
        hinge_world.step()
        anchor_position = (
            np.asarray(hinge_body.translation, dtype=float)
            + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
        )
        max_anchor_residual = max(
            max_anchor_residual,
            float(np.linalg.norm(anchor_position - hinge_world_anchor)),
        )
    assert max_anchor_residual > 1e-4

    hinge_body.parent_joint.velocity = [0.0] * 6
    hinge.break_force = 1.0e12
    hinge.reset_breakage()

    assert not hinge.is_broken

    hinge_world.step()

    assert not hinge.is_broken
    assert hinge.child_link == hinge_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = hinge.parent_link
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        [0.0, 0.0, 1.0]
    )
    hinge_reset_anchor_position = (
        np.asarray(hinge_body.translation, dtype=float)
        + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
    )
    assert np.linalg.norm(hinge_reset_anchor_position - hinge_world_anchor) < 1e-6
    assert (
        np.linalg.norm(
            np.asarray(hinge_body.rotation, dtype=float) @ np.array([0.0, 0.0, 1.0])
            - np.array([0.0, 0.0, 1.0])
        )
        < 1e-6
    )
    hinge_reset_yaw = math.atan2(
        np.asarray(hinge_body.rotation, dtype=float)[1, 0],
        np.asarray(hinge_body.rotation, dtype=float)[0, 0],
    )
    assert hinge_reset_yaw > hinge_yaw


def test_simulation_world_articulated_non_cardinal_prismatic_reset_from_python():
    sx = _simulation()

    world, _, body = _floating_link_world(
        sx, "python_non_cardinal_world_breakable_slider"
    )
    body.parent_joint.position = [0.2, -0.15, 0.05, 0.0, 0.0, 0.0]

    slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    slider_axis /= np.linalg.norm(slider_axis)
    child_anchor = np.array([0.0, 0.2, 0.1])
    world_anchor = (
        np.asarray(body.translation, dtype=float)
        + np.asarray(body.rotation, dtype=float) @ child_anchor
    )
    slider = world.add_articulated_prismatic_joint(
        "non_cardinal_world_slider",
        body,
        axis=slider_axis.tolist(),
        world_anchor=world_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.child_link == body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    target_speed = 0.3
    slider.command_velocity = [target_speed]
    slider.set_effort_limits([-1000.0], [1000.0])
    slider.break_force = 1e-18

    def anchor_delta() -> np.ndarray:
        return (
            np.asarray(body.translation, dtype=float)
            + np.asarray(body.rotation, dtype=float) @ child_anchor
            - world_anchor
        )

    def orthogonal_anchor_residual() -> float:
        delta = anchor_delta()
        return float(np.linalg.norm(delta - float(delta @ slider_axis) * slider_axis))

    world.enter_simulation_mode()
    world.step()

    assert slider.is_broken
    first_axis_position = float(anchor_delta() @ slider_axis)
    assert first_axis_position == pytest.approx(
        target_speed * world.time_step, abs=1e-6
    )
    assert orthogonal_anchor_residual() < 1e-6
    assert np.linalg.norm(np.asarray(body.rotation, dtype=float) - np.eye(3)) < 1e-6

    lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    lateral_force -= float(lateral_force @ slider_axis) * slider_axis
    lateral_force /= np.linalg.norm(lateral_force)

    max_broken_orthogonal_residual = 0.0
    for _ in range(20):
        body.apply_force((4.0 * lateral_force).tolist())
        world.step()
        max_broken_orthogonal_residual = max(
            max_broken_orthogonal_residual, orthogonal_anchor_residual()
        )
    assert max_broken_orthogonal_residual > 1e-4
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.child_link == body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )

    body.parent_joint.velocity = [0.0] * 6
    slider.command_velocity = [-target_speed]
    slider.break_force = 1.0e12
    slider.reset_breakage()

    assert not slider.is_broken
    axis_position_before_reset = float(anchor_delta() @ slider_axis)

    for _ in range(6):
        world.step()

    assert not slider.is_broken
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.child_link == body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    assert orthogonal_anchor_residual() < 1e-6
    assert np.linalg.norm(np.asarray(body.rotation, dtype=float) - np.eye(3)) < 1e-6
    assert float(anchor_delta() @ slider_axis) < axis_position_before_reset - 1e-3


def test_simulation_world_articulated_non_cardinal_revolute_reset_from_python():
    sx = _simulation()

    world, _, body = _floating_link_world(
        sx, "python_non_cardinal_world_breakable_hinge"
    )
    body.parent_joint.position = [0.1, -0.2, 0.15, 0.0, 0.0, 0.0]

    hinge_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    hinge_axis /= np.linalg.norm(hinge_axis)
    child_anchor = np.array([-0.15, 0.05, 0.02])
    world_anchor = (
        np.asarray(body.translation, dtype=float)
        + np.asarray(body.rotation, dtype=float) @ child_anchor
    )
    hinge = world.add_articulated_revolute_joint(
        "non_cardinal_world_hinge",
        body,
        axis=hinge_axis.tolist(),
        world_anchor=world_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.child_link == body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = hinge.parent_link
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    target_speed = 0.4
    hinge.command_velocity = [target_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    def anchor_residual() -> float:
        anchor_position = (
            np.asarray(body.translation, dtype=float)
            + np.asarray(body.rotation, dtype=float) @ child_anchor
        )
        return float(np.linalg.norm(anchor_position - world_anchor))

    def signed_axis_angle() -> float:
        rotation = np.asarray(body.rotation, dtype=float)
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def axis_tilt() -> float:
        rotation = np.asarray(body.rotation, dtype=float)
        return float(np.linalg.norm(rotation @ hinge_axis - hinge_axis))

    world.enter_simulation_mode()
    world.step()

    assert hinge.is_broken
    first_axis_angle = signed_axis_angle()
    assert first_axis_angle == pytest.approx(target_speed * world.time_step, abs=1e-6)
    assert anchor_residual() < 1e-6
    assert axis_tilt() < 1e-6

    lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    lateral_force -= float(lateral_force @ hinge_axis) * hinge_axis
    lateral_force /= np.linalg.norm(lateral_force)
    off_axis_force_point = child_anchor + np.array([0.3, 0.0, 0.0])

    max_broken_anchor_residual = 0.0
    for _ in range(20):
        body.apply_force((4.0 * lateral_force).tolist(), off_axis_force_point.tolist())
        world.step()
        max_broken_anchor_residual = max(
            max_broken_anchor_residual, anchor_residual()
        )
    assert max_broken_anchor_residual > 1e-4
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.child_link == body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = hinge.parent_link
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )

    body.parent_joint.velocity = [0.0] * 6
    hinge.command_velocity = [-target_speed]
    hinge.break_force = 1.0e12
    hinge.reset_breakage()

    assert not hinge.is_broken
    axis_angle_before_reset = signed_axis_angle()

    for _ in range(6):
        world.step()

    assert not hinge.is_broken
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.child_link == body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = hinge.parent_link
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    assert anchor_residual() < 1e-6
    assert axis_tilt() < 1e-6
    assert signed_axis_angle() < axis_angle_before_reset - 1e-3


def test_simulation_world_articulated_non_cardinal_pair_motors_reset_from_python():
    sx = _simulation()

    slider_world, slider_parent, slider_child = _floating_link_pair_world(
        sx, "python_non_cardinal_pair_breakable_slider"
    )
    slider_child.parent_joint.position = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]

    slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    slider_axis /= np.linalg.norm(slider_axis)
    parent_anchor = np.array([0.2, 0.1, 0.0])
    child_anchor = np.array([-0.1, 0.1, 0.0])
    slider = slider_world.add_articulated_prismatic_joint(
        "non_cardinal_pair_slider",
        slider_parent,
        slider_child,
        axis=slider_axis.tolist(),
        parent_anchor=parent_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.parent_link == slider_parent
    assert slider.child_link == slider_child
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1000.0], [1000.0])
    slider.break_force = 1e-18

    def slider_anchor_delta() -> np.ndarray:
        parent_anchor_world = (
            np.asarray(slider_parent.translation, dtype=float)
            + np.asarray(slider_parent.rotation, dtype=float) @ parent_anchor
        )
        child_anchor_world = (
            np.asarray(slider_child.translation, dtype=float)
            + np.asarray(slider_child.rotation, dtype=float) @ child_anchor
        )
        return child_anchor_world - parent_anchor_world

    def slider_orthogonal_residual() -> float:
        delta = slider_anchor_delta()
        return float(np.linalg.norm(delta - float(delta @ slider_axis) * slider_axis))

    def slider_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(slider_parent.rotation, dtype=float).T
            @ np.asarray(slider_child.rotation, dtype=float)
        )
        return float(np.linalg.norm(relative_rotation - np.eye(3)))

    slider_world.enter_simulation_mode()
    slider_world.step()

    assert slider.is_broken
    slider_first_delta = slider_anchor_delta()
    assert float(slider_first_delta @ slider_axis) == pytest.approx(
        slider_speed * slider_world.time_step, abs=1e-6
    )
    assert slider_orthogonal_residual() < 1e-6
    assert slider_relative_rotation_error() < 1e-6

    lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    lateral_force -= float(lateral_force @ slider_axis) * slider_axis
    lateral_force /= np.linalg.norm(lateral_force)

    max_slider_broken_orthogonal_residual = 0.0
    for _ in range(20):
        slider_parent.apply_force((-4.0 * lateral_force).tolist(), parent_anchor.tolist())
        slider_child.apply_force((4.0 * lateral_force).tolist(), child_anchor.tolist())
        slider_world.step()
        max_slider_broken_orthogonal_residual = max(
            max_slider_broken_orthogonal_residual, slider_orthogonal_residual()
        )
    assert max_slider_broken_orthogonal_residual > 1e-4
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.parent_link == slider_parent
    assert slider.child_link == slider_child
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )

    slider_parent.parent_joint.velocity = [0.0] * 6
    slider_child.parent_joint.velocity = [0.0] * 6
    slider.command_velocity = [-slider_speed]
    slider.break_force = 1.0e12
    slider.reset_breakage()

    assert not slider.is_broken
    slider_axis_position_before_reset = float(slider_anchor_delta() @ slider_axis)

    for _ in range(6):
        slider_world.step()

    assert not slider.is_broken
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.parent_link == slider_parent
    assert slider.child_link == slider_child
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider_reset_orthogonal_residual = slider_orthogonal_residual()
    assert (
        slider_reset_orthogonal_residual
        < max_slider_broken_orthogonal_residual * 0.05
    )
    assert slider_reset_orthogonal_residual < 1e-3
    assert slider_relative_rotation_error() < 1e-6
    assert (
        float(slider_anchor_delta() @ slider_axis)
        < slider_axis_position_before_reset - 1e-3
    )

    hinge_world, hinge_parent, hinge_child = _floating_link_pair_world(
        sx, "python_non_cardinal_pair_breakable_hinge"
    )
    hinge_child.parent_joint.position = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]

    hinge_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    hinge_axis /= np.linalg.norm(hinge_axis)
    hinge = hinge_world.add_articulated_revolute_joint(
        "non_cardinal_pair_hinge",
        hinge_parent,
        hinge_child,
        axis=hinge_axis.tolist(),
        parent_anchor=parent_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == hinge_parent
    assert hinge.child_link == hinge_child
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    def hinge_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(hinge_parent.translation, dtype=float)
            + np.asarray(hinge_parent.rotation, dtype=float) @ parent_anchor
        )
        child_anchor_world = (
            np.asarray(hinge_child.translation, dtype=float)
            + np.asarray(hinge_child.rotation, dtype=float) @ child_anchor
        )
        return float(np.linalg.norm(child_anchor_world - parent_anchor_world))

    def hinge_relative_rotation() -> np.ndarray:
        return (
            np.asarray(hinge_parent.rotation, dtype=float).T
            @ np.asarray(hinge_child.rotation, dtype=float)
        )

    def hinge_signed_axis_angle() -> float:
        rotation = hinge_relative_rotation()
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(hinge_parent.rotation, dtype=float) @ hinge_axis
                - np.asarray(hinge_child.rotation, dtype=float) @ hinge_axis
            )
        )

    hinge_world.enter_simulation_mode()
    hinge_world.step()

    assert hinge.is_broken
    first_hinge_axis_angle = hinge_signed_axis_angle()
    assert first_hinge_axis_angle == pytest.approx(
        hinge_speed * hinge_world.time_step, abs=1e-6
    )
    assert hinge_anchor_residual() < 1e-6
    assert hinge_axis_tilt() < 1e-6

    hinge_lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    hinge_lateral_force -= float(hinge_lateral_force @ hinge_axis) * hinge_axis
    hinge_lateral_force /= np.linalg.norm(hinge_lateral_force)
    parent_force_point = parent_anchor + np.array([0.3, 0.0, 0.0])
    child_force_point = child_anchor - np.array([0.3, 0.0, 0.0])

    max_hinge_broken_anchor_residual = 0.0
    max_hinge_broken_axis_tilt = 0.0
    for _ in range(20):
        hinge_parent.apply_force(
            (-4.0 * hinge_lateral_force).tolist(), parent_force_point.tolist()
        )
        hinge_child.apply_force(
            (4.0 * hinge_lateral_force).tolist(), child_force_point.tolist()
        )
        hinge_world.step()
        max_hinge_broken_anchor_residual = max(
            max_hinge_broken_anchor_residual, hinge_anchor_residual()
        )
        max_hinge_broken_axis_tilt = max(
            max_hinge_broken_axis_tilt, hinge_axis_tilt()
        )
    assert max_hinge_broken_anchor_residual > 1e-4
    assert max_hinge_broken_axis_tilt > 1e-4
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == hinge_parent
    assert hinge.child_link == hinge_child
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )

    hinge_parent.parent_joint.velocity = [0.0] * 6
    hinge_child.parent_joint.velocity = [0.0] * 6
    hinge.command_velocity = [-hinge_speed]
    hinge.break_force = 1.0e12
    hinge.reset_breakage()

    assert not hinge.is_broken
    hinge_axis_angle_before_reset = hinge_signed_axis_angle()

    for _ in range(6):
        hinge_world.step()

    assert not hinge.is_broken
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == hinge_parent
    assert hinge.child_link == hinge_child
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    assert hinge_anchor_residual() < 1e-6
    assert hinge_axis_tilt() < max_hinge_broken_axis_tilt * 0.01
    assert hinge_axis_tilt() < 1e-3
    assert hinge_signed_axis_angle() < hinge_axis_angle_before_reset - 1e-3


def test_simulation_world_articulated_non_cardinal_off_origin_tiny_effort_motors_from_python():
    sx = _simulation()

    slider_world, _, slider_body = _floating_link_world(
        sx, "python_non_cardinal_world_tiny_slider"
    )
    slider_body.parent_joint.position = [0.2, -0.15, 0.05, 0.0, 0.0, 0.0]

    slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    slider_axis /= np.linalg.norm(slider_axis)
    slider_child_anchor = np.array([0.1, 0.2, -0.1])
    slider_world_anchor = (
        np.asarray(slider_body.translation, dtype=float)
        + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
    )
    slider = slider_world.add_articulated_prismatic_joint(
        "non_cardinal_world_tiny_slider",
        slider_body,
        axis=slider_axis.tolist(),
        world_anchor=slider_world_anchor.tolist(),
        child_anchor=slider_child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.child_link == slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = slider.parent_link
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1e-9], [1e-9])
    assert slider.effort_lower_limits.tolist() == pytest.approx([-1e-9])
    assert slider.effort_upper_limits.tolist() == pytest.approx([1e-9])

    def slider_anchor_delta() -> np.ndarray:
        return (
            np.asarray(slider_body.translation, dtype=float)
            + np.asarray(slider_body.rotation, dtype=float) @ slider_child_anchor
            - slider_world_anchor
        )

    def slider_orthogonal_residual() -> float:
        delta = slider_anchor_delta()
        return float(np.linalg.norm(delta - float(delta @ slider_axis) * slider_axis))

    slider_world.enter_simulation_mode()

    max_slider_orthogonal_residual = 0.0
    max_slider_rotation_error = 0.0
    steps = 20
    for _ in range(steps):
        slider_world.step()
        max_slider_orthogonal_residual = max(
            max_slider_orthogonal_residual, slider_orthogonal_residual()
        )
        max_slider_rotation_error = max(
            max_slider_rotation_error,
            float(np.linalg.norm(np.asarray(slider_body.rotation, dtype=float) - np.eye(3))),
        )

    assert not slider.is_broken
    assert max_slider_orthogonal_residual < 1e-6
    assert max_slider_rotation_error < 1e-6
    assert abs(float(slider_anchor_delta() @ slider_axis)) < (
        0.01 * slider_speed * slider_world.time_step * steps
    )

    world_hinge_world, _, world_hinge_body = _floating_link_world(
        sx, "python_non_cardinal_world_tiny_hinge"
    )
    world_hinge_body.parent_joint.position = [0.1, -0.2, 0.15, 0.0, 0.0, 0.0]

    world_hinge_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    world_hinge_axis /= np.linalg.norm(world_hinge_axis)
    world_hinge_child_anchor = np.array([0.2, 0.1, 0.0])
    world_hinge_anchor = (
        np.asarray(world_hinge_body.translation, dtype=float)
        + np.asarray(world_hinge_body.rotation, dtype=float) @ world_hinge_child_anchor
    )
    world_hinge = world_hinge_world.add_articulated_revolute_joint(
        "non_cardinal_world_tiny_hinge",
        world_hinge_body,
        axis=world_hinge_axis.tolist(),
        world_anchor=world_hinge_anchor.tolist(),
        child_anchor=world_hinge_child_anchor.tolist(),
    )
    assert world_hinge.type == sx.JointType.REVOLUTE
    assert world_hinge.num_dofs == 1
    assert world_hinge.child_link == world_hinge_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_hinge.parent_link
    assert np.asarray(world_hinge.axis, dtype=float).tolist() == pytest.approx(
        world_hinge_axis.tolist()
    )
    world_hinge.actuator_type = sx.ActuatorType.VELOCITY
    world_hinge_speed = 0.4
    world_hinge.command_velocity = [world_hinge_speed]
    world_hinge.set_effort_limits([-1e-9], [1e-9])
    assert world_hinge.effort_lower_limits.tolist() == pytest.approx([-1e-9])
    assert world_hinge.effort_upper_limits.tolist() == pytest.approx([1e-9])

    def world_hinge_anchor_residual() -> float:
        anchor_position = (
            np.asarray(world_hinge_body.translation, dtype=float)
            + np.asarray(world_hinge_body.rotation, dtype=float)
            @ world_hinge_child_anchor
        )
        return float(np.linalg.norm(anchor_position - world_hinge_anchor))

    def world_hinge_signed_axis_angle() -> float:
        rotation = np.asarray(world_hinge_body.rotation, dtype=float)
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(world_hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def world_hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(world_hinge_body.rotation, dtype=float) @ world_hinge_axis
                - world_hinge_axis
            )
        )

    world_hinge_world.enter_simulation_mode()

    max_world_hinge_anchor_residual = 0.0
    max_world_hinge_axis_tilt = 0.0
    for _ in range(steps):
        world_hinge_world.step()
        max_world_hinge_anchor_residual = max(
            max_world_hinge_anchor_residual, world_hinge_anchor_residual()
        )
        max_world_hinge_axis_tilt = max(
            max_world_hinge_axis_tilt, world_hinge_axis_tilt()
        )

    assert not world_hinge.is_broken
    assert max_world_hinge_anchor_residual < 1e-6
    assert max_world_hinge_axis_tilt < 1e-3
    assert abs(world_hinge_signed_axis_angle()) < (
        0.01 * world_hinge_speed * world_hinge_world.time_step * steps
    )

    pair_slider_world, pair_slider_parent, pair_slider_child = (
        _floating_link_pair_world(sx, "python_non_cardinal_pair_tiny_slider")
    )
    pair_slider_parent_anchor = np.array([0.2, 0.1, 0.0])
    pair_slider_child_anchor = np.array([-0.1, 0.1, 0.0])
    pair_slider_child.parent_joint.position = (
        pair_slider_parent_anchor - pair_slider_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]

    pair_slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    pair_slider_axis /= np.linalg.norm(pair_slider_axis)
    pair_slider = pair_slider_world.add_articulated_prismatic_joint(
        "non_cardinal_pair_tiny_slider",
        pair_slider_parent,
        pair_slider_child,
        axis=pair_slider_axis.tolist(),
        parent_anchor=pair_slider_parent_anchor.tolist(),
        child_anchor=pair_slider_child_anchor.tolist(),
    )
    assert pair_slider.type == sx.JointType.PRISMATIC
    assert pair_slider.num_dofs == 1
    assert pair_slider.parent_link == pair_slider_parent
    assert pair_slider.child_link == pair_slider_child
    assert np.asarray(pair_slider.axis, dtype=float).tolist() == pytest.approx(
        pair_slider_axis.tolist()
    )
    pair_slider.actuator_type = sx.ActuatorType.VELOCITY
    pair_slider_speed = 0.3
    pair_slider.command_velocity = [pair_slider_speed]
    pair_slider.set_effort_limits([-1e-9], [1e-9])
    assert pair_slider.effort_lower_limits.tolist() == pytest.approx([-1e-9])
    assert pair_slider.effort_upper_limits.tolist() == pytest.approx([1e-9])

    def pair_slider_anchor_delta() -> np.ndarray:
        parent_anchor_world = (
            np.asarray(pair_slider_parent.translation, dtype=float)
            + np.asarray(pair_slider_parent.rotation, dtype=float)
            @ pair_slider_parent_anchor
        )
        child_anchor_world = (
            np.asarray(pair_slider_child.translation, dtype=float)
            + np.asarray(pair_slider_child.rotation, dtype=float)
            @ pair_slider_child_anchor
        )
        return child_anchor_world - parent_anchor_world

    def pair_slider_orthogonal_residual() -> float:
        delta = pair_slider_anchor_delta()
        return float(
            np.linalg.norm(delta - float(delta @ pair_slider_axis) * pair_slider_axis)
        )

    def pair_slider_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(pair_slider_parent.rotation, dtype=float).T
            @ np.asarray(pair_slider_child.rotation, dtype=float)
        )
        return float(np.linalg.norm(relative_rotation - np.eye(3)))

    pair_slider_world.enter_simulation_mode()

    max_pair_slider_orthogonal_residual = 0.0
    max_pair_slider_rotation_error = 0.0
    for _ in range(steps):
        pair_slider_world.step()
        max_pair_slider_orthogonal_residual = max(
            max_pair_slider_orthogonal_residual,
            pair_slider_orthogonal_residual(),
        )
        max_pair_slider_rotation_error = max(
            max_pair_slider_rotation_error,
            pair_slider_relative_rotation_error(),
        )

    assert not pair_slider.is_broken
    assert max_pair_slider_orthogonal_residual < 1e-6
    assert max_pair_slider_rotation_error < 1e-6
    assert abs(float(pair_slider_anchor_delta() @ pair_slider_axis)) < (
        0.01 * pair_slider_speed * pair_slider_world.time_step * steps
    )

    hinge_world, hinge_parent, hinge_child = _floating_link_pair_world(
        sx, "python_non_cardinal_pair_tiny_hinge"
    )
    hinge_parent_anchor = np.array([0.2, 0.0, 0.0])
    hinge_child_anchor = np.array([-0.1, 0.0, 0.0])
    hinge_child.parent_joint.position = (
        hinge_parent_anchor - hinge_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]

    hinge_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    hinge_axis /= np.linalg.norm(hinge_axis)
    hinge = hinge_world.add_articulated_revolute_joint(
        "non_cardinal_pair_tiny_hinge",
        hinge_parent,
        hinge_child,
        axis=hinge_axis.tolist(),
        parent_anchor=hinge_parent_anchor.tolist(),
        child_anchor=hinge_child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == hinge_parent
    assert hinge.child_link == hinge_child
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1e-9], [1e-9])
    assert hinge.effort_lower_limits.tolist() == pytest.approx([-1e-9])
    assert hinge.effort_upper_limits.tolist() == pytest.approx([1e-9])

    def hinge_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(hinge_parent.translation, dtype=float)
            + np.asarray(hinge_parent.rotation, dtype=float) @ hinge_parent_anchor
        )
        child_anchor_world = (
            np.asarray(hinge_child.translation, dtype=float)
            + np.asarray(hinge_child.rotation, dtype=float) @ hinge_child_anchor
        )
        return float(np.linalg.norm(child_anchor_world - parent_anchor_world))

    def hinge_relative_rotation() -> np.ndarray:
        return (
            np.asarray(hinge_parent.rotation, dtype=float).T
            @ np.asarray(hinge_child.rotation, dtype=float)
        )

    def hinge_signed_axis_angle() -> float:
        rotation = hinge_relative_rotation()
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(hinge_parent.rotation, dtype=float) @ hinge_axis
                - np.asarray(hinge_child.rotation, dtype=float) @ hinge_axis
            )
        )

    hinge_world.enter_simulation_mode()

    max_hinge_anchor_residual = 0.0
    max_hinge_axis_tilt = 0.0
    for _ in range(steps):
        hinge_world.step()
        max_hinge_anchor_residual = max(
            max_hinge_anchor_residual, hinge_anchor_residual()
        )
        max_hinge_axis_tilt = max(max_hinge_axis_tilt, hinge_axis_tilt())

    assert not hinge.is_broken
    assert max_hinge_anchor_residual < 1e-6
    assert max_hinge_axis_tilt < 1e-3
    assert abs(hinge_signed_axis_angle()) < (
        0.01 * hinge_speed * hinge_world.time_step * steps
    )


def test_simulation_world_articulated_binary_roundtrip_from_python(tmp_path: Path):
    sx = _simulation()

    pair_world, pair_parent, pair_child = _floating_link_pair_world(
        sx, "python_serialized_pair_slider"
    )
    pair_parent_anchor = np.array([0.2, 0.1, 0.0])
    pair_child_anchor = np.array([-0.1, 0.1, 0.0])
    pair_child.parent_joint.position = (
        pair_parent_anchor - pair_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]

    pair_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    pair_axis /= np.linalg.norm(pair_axis)
    pair_slider = pair_world.add_articulated_prismatic_joint(
        "serialized_pair_slider",
        pair_parent,
        pair_child,
        axis=pair_axis.tolist(),
        parent_anchor=pair_parent_anchor.tolist(),
        child_anchor=pair_child_anchor.tolist(),
    )
    pair_slider.actuator_type = sx.ActuatorType.VELOCITY
    pair_speed = 0.3
    pair_slider.command_velocity = [pair_speed]
    pair_slider.set_effort_limits([-1000.0], [1000.0])
    pair_slider.break_force = 1e-18

    pair_world.enter_simulation_mode()
    pair_world.step()
    assert pair_slider.is_broken

    pair_path = tmp_path / "articulated_pair_slider.bin"
    pair_world.save_binary(pair_path)

    restored_pair_world = sx.World()
    restored_pair_world.load_binary(pair_path)
    restored_pair_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_pair_arm = restored_pair_world.get_multibody(
        "python_serialized_pair_slider"
    )
    assert restored_pair_arm is not None
    restored_pair_parent = restored_pair_arm.get_link("parent")
    restored_pair_child = restored_pair_arm.get_link("child")
    restored_pair_slider = restored_pair_world.get_articulated_joint(
        "serialized_pair_slider"
    )
    assert restored_pair_parent is not None
    assert restored_pair_child is not None
    assert restored_pair_slider is not None
    assert restored_pair_slider.is_broken
    assert restored_pair_slider.type == sx.JointType.PRISMATIC
    assert restored_pair_slider.num_dofs == 1
    assert restored_pair_slider.parent_link == restored_pair_parent
    assert restored_pair_slider.child_link == restored_pair_child
    assert np.asarray(restored_pair_slider.axis, dtype=float).tolist() == pytest.approx(
        pair_axis.tolist()
    )
    assert restored_pair_slider.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_pair_slider.command_velocity.tolist() == pytest.approx(
        [pair_speed]
    )
    assert restored_pair_slider.effort_lower_limits.tolist() == pytest.approx(
        [-1000.0]
    )
    assert restored_pair_slider.effort_upper_limits.tolist() == pytest.approx(
        [1000.0]
    )

    def pair_anchor_delta() -> np.ndarray:
        parent_anchor_world = (
            np.asarray(restored_pair_parent.translation, dtype=float)
            + np.asarray(restored_pair_parent.rotation, dtype=float)
            @ pair_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_pair_child.translation, dtype=float)
            + np.asarray(restored_pair_child.rotation, dtype=float) @ pair_child_anchor
        )
        return child_anchor_world - parent_anchor_world

    def pair_orthogonal_residual() -> float:
        delta = pair_anchor_delta()
        return float(np.linalg.norm(delta - float(delta @ pair_axis) * pair_axis))

    def pair_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(restored_pair_parent.rotation, dtype=float).T
            @ np.asarray(restored_pair_child.rotation, dtype=float)
        )
        return float(np.linalg.norm(relative_rotation - np.eye(3)))

    pair_lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    pair_lateral_force -= float(pair_lateral_force @ pair_axis) * pair_axis
    pair_lateral_force /= np.linalg.norm(pair_lateral_force)

    max_pair_broken_orthogonal_residual = 0.0
    for _ in range(20):
        restored_pair_parent.apply_force(
            (-4.0 * pair_lateral_force).tolist(),
            pair_parent_anchor.tolist(),
        )
        restored_pair_child.apply_force(
            (4.0 * pair_lateral_force).tolist(),
            pair_child_anchor.tolist(),
        )
        restored_pair_world.step()
        max_pair_broken_orthogonal_residual = max(
            max_pair_broken_orthogonal_residual,
            pair_orthogonal_residual(),
        )
    assert max_pair_broken_orthogonal_residual > 1e-4

    restored_pair_parent.parent_joint.velocity = [0.0] * 6
    restored_pair_child.parent_joint.velocity = [0.0] * 6
    restored_pair_slider.command_velocity = [-pair_speed]
    restored_pair_slider.break_force = 1.0e12
    restored_pair_slider.reset_breakage()
    pair_axis_position_before_reset = float(pair_anchor_delta() @ pair_axis)

    for _ in range(8):
        restored_pair_world.step()

    assert not restored_pair_slider.is_broken
    assert restored_pair_slider.type == sx.JointType.PRISMATIC
    assert restored_pair_slider.num_dofs == 1
    assert restored_pair_slider.parent_link == restored_pair_parent
    assert restored_pair_slider.child_link == restored_pair_child
    assert np.asarray(restored_pair_slider.axis, dtype=float).tolist() == pytest.approx(
        pair_axis.tolist()
    )
    assert (
        pair_orthogonal_residual()
        < max_pair_broken_orthogonal_residual * 0.05
    )
    assert pair_orthogonal_residual() < 1e-3
    assert pair_relative_rotation_error() < 1e-6
    assert (
        float(pair_anchor_delta() @ pair_axis)
        < pair_axis_position_before_reset - 1e-3
    )

    hinge_world, _, hinge_body = _floating_link_world(
        sx, "python_serialized_world_hinge"
    )
    hinge_body.parent_joint.position = [0.15, -0.05, 0.0, 0.0, 0.0, 0.0]
    hinge_axis = np.array([1.0, 2.0, 3.0], dtype=float)
    hinge_axis /= np.linalg.norm(hinge_axis)
    hinge_child_anchor = np.array([0.2, 0.1, 0.0])
    hinge_world_anchor = (
        np.asarray(hinge_body.translation, dtype=float)
        + np.asarray(hinge_body.rotation, dtype=float) @ hinge_child_anchor
    )
    hinge = hinge_world.add_articulated_revolute_joint(
        "serialized_world_hinge",
        hinge_body,
        axis=hinge_axis.tolist(),
        world_anchor=hinge_world_anchor.tolist(),
        child_anchor=hinge_child_anchor.tolist(),
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    hinge_world.enter_simulation_mode()
    hinge_world.step()
    assert hinge.is_broken

    hinge_path = tmp_path / "articulated_world_hinge.bin"
    hinge_world.save_binary(hinge_path)

    restored_hinge_world = sx.World()
    restored_hinge_world.load_binary(hinge_path)
    restored_hinge_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_hinge_arm = restored_hinge_world.get_multibody(
        "python_serialized_world_hinge"
    )
    assert restored_hinge_arm is not None
    restored_hinge_body = restored_hinge_arm.get_link("body")
    restored_hinge = restored_hinge_world.get_articulated_joint(
        "serialized_world_hinge"
    )
    assert restored_hinge_body is not None
    assert restored_hinge is not None
    assert restored_hinge.is_broken
    assert restored_hinge.type == sx.JointType.REVOLUTE
    assert restored_hinge.num_dofs == 1
    assert restored_hinge.child_link == restored_hinge_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_hinge.parent_link
    assert np.asarray(restored_hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    assert restored_hinge.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_hinge.command_velocity.tolist() == pytest.approx([hinge_speed])
    assert restored_hinge.effort_lower_limits.tolist() == pytest.approx([-1000.0])
    assert restored_hinge.effort_upper_limits.tolist() == pytest.approx([1000.0])

    def hinge_anchor_residual() -> float:
        anchor_position = (
            np.asarray(restored_hinge_body.translation, dtype=float)
            + np.asarray(restored_hinge_body.rotation, dtype=float)
            @ hinge_child_anchor
        )
        return float(np.linalg.norm(anchor_position - hinge_world_anchor))

    def hinge_signed_axis_angle() -> float:
        rotation = np.asarray(restored_hinge_body.rotation, dtype=float)
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_hinge_body.rotation, dtype=float) @ hinge_axis
                - hinge_axis
            )
        )

    hinge_lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    hinge_lateral_force -= float(hinge_lateral_force @ hinge_axis) * hinge_axis
    hinge_lateral_force /= np.linalg.norm(hinge_lateral_force)
    hinge_force_point = hinge_child_anchor + np.array([0.4, 0.0, 0.0])

    max_hinge_broken_anchor_residual = 0.0
    max_hinge_broken_axis_tilt = 0.0
    for _ in range(20):
        restored_hinge_body.apply_force(
            (4.0 * hinge_lateral_force).tolist(),
            hinge_force_point.tolist(),
        )
        restored_hinge_world.step()
        max_hinge_broken_anchor_residual = max(
            max_hinge_broken_anchor_residual,
            hinge_anchor_residual(),
        )
        max_hinge_broken_axis_tilt = max(
            max_hinge_broken_axis_tilt,
            hinge_axis_tilt(),
        )
    assert max_hinge_broken_anchor_residual > 1e-4
    assert max_hinge_broken_axis_tilt > 1e-4

    restored_hinge_body.parent_joint.velocity = [0.0] * 6
    restored_hinge.command_velocity = [-hinge_speed]
    restored_hinge.break_force = 1.0e12
    restored_hinge.reset_breakage()
    hinge_angle_before_reset = hinge_signed_axis_angle()

    for _ in range(8):
        restored_hinge_world.step()

    assert not restored_hinge.is_broken
    assert restored_hinge.type == sx.JointType.REVOLUTE
    assert restored_hinge.num_dofs == 1
    assert restored_hinge.child_link == restored_hinge_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_hinge.parent_link
    assert np.asarray(restored_hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    assert hinge_anchor_residual() < max_hinge_broken_anchor_residual * 0.05
    assert hinge_anchor_residual() < 1e-3
    assert hinge_axis_tilt() < max_hinge_broken_axis_tilt * 0.05
    assert hinge_axis_tilt() < 1e-3
    assert hinge_signed_axis_angle() < hinge_angle_before_reset - 1e-3


def test_simulation_world_articulated_binary_roundtrip_from_python_completes_one_dof_endpoint_types(
    tmp_path: Path,
):
    sx = _simulation()

    pair_hinge_world, pair_hinge_parent, pair_hinge_child = _floating_link_pair_world(
        sx, "python_serialized_pair_hinge_broken"
    )
    pair_hinge_parent_anchor = np.array([0.2, 0.1, 0.0])
    pair_hinge_child_anchor = np.array([-0.1, 0.1, 0.0])
    pair_hinge_child.parent_joint.position = (
        pair_hinge_parent_anchor - pair_hinge_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    pair_hinge_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    pair_hinge_axis /= np.linalg.norm(pair_hinge_axis)
    pair_hinge = pair_hinge_world.add_articulated_revolute_joint(
        "serialized_pair_hinge_broken",
        pair_hinge_parent,
        pair_hinge_child,
        axis=pair_hinge_axis.tolist(),
        parent_anchor=pair_hinge_parent_anchor.tolist(),
        child_anchor=pair_hinge_child_anchor.tolist(),
    )
    pair_hinge.actuator_type = sx.ActuatorType.VELOCITY
    pair_hinge_speed = 0.35
    pair_hinge.command_velocity = [pair_hinge_speed]
    pair_hinge.set_effort_limits([-1000.0], [1000.0])
    pair_hinge.break_force = 1e-18

    pair_hinge_world.enter_simulation_mode()
    pair_hinge_world.step()
    assert pair_hinge.is_broken

    pair_hinge_path = tmp_path / "articulated_pair_hinge_broken.bin"
    pair_hinge_world.save_binary(pair_hinge_path)

    restored_pair_hinge_world = sx.World()
    restored_pair_hinge_world.load_binary(pair_hinge_path)
    restored_pair_hinge_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_pair_hinge_arm = restored_pair_hinge_world.get_multibody(
        "python_serialized_pair_hinge_broken"
    )
    assert restored_pair_hinge_arm is not None
    restored_pair_hinge_parent = restored_pair_hinge_arm.get_link("parent")
    restored_pair_hinge_child = restored_pair_hinge_arm.get_link("child")
    restored_pair_hinge = restored_pair_hinge_world.get_articulated_joint(
        "serialized_pair_hinge_broken"
    )
    assert restored_pair_hinge_parent is not None
    assert restored_pair_hinge_child is not None
    assert restored_pair_hinge is not None
    assert restored_pair_hinge.is_broken
    assert restored_pair_hinge.type == sx.JointType.REVOLUTE
    assert restored_pair_hinge.num_dofs == 1
    assert restored_pair_hinge.parent_link == restored_pair_hinge_parent
    assert restored_pair_hinge.child_link == restored_pair_hinge_child
    assert np.asarray(restored_pair_hinge.axis, dtype=float).tolist() == pytest.approx(
        pair_hinge_axis.tolist()
    )
    assert restored_pair_hinge.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_pair_hinge.command_velocity.tolist() == pytest.approx(
        [pair_hinge_speed]
    )
    assert restored_pair_hinge.effort_lower_limits.tolist() == pytest.approx(
        [-1000.0]
    )
    assert restored_pair_hinge.effort_upper_limits.tolist() == pytest.approx(
        [1000.0]
    )

    def pair_hinge_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(restored_pair_hinge_parent.translation, dtype=float)
            + np.asarray(restored_pair_hinge_parent.rotation, dtype=float)
            @ pair_hinge_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_pair_hinge_child.translation, dtype=float)
            + np.asarray(restored_pair_hinge_child.rotation, dtype=float)
            @ pair_hinge_child_anchor
        )
        return float(np.linalg.norm(child_anchor_world - parent_anchor_world))

    def pair_hinge_relative_rotation() -> np.ndarray:
        return (
            np.asarray(restored_pair_hinge_parent.rotation, dtype=float).T
            @ np.asarray(restored_pair_hinge_child.rotation, dtype=float)
        )

    def pair_hinge_signed_axis_angle() -> float:
        rotation = pair_hinge_relative_rotation()
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(pair_hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def pair_hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_pair_hinge_parent.rotation, dtype=float)
                @ pair_hinge_axis
                - np.asarray(restored_pair_hinge_child.rotation, dtype=float)
                @ pair_hinge_axis
            )
        )

    pair_hinge_lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    pair_hinge_lateral_force -= (
        float(pair_hinge_lateral_force @ pair_hinge_axis) * pair_hinge_axis
    )
    pair_hinge_lateral_force /= np.linalg.norm(pair_hinge_lateral_force)
    pair_hinge_parent_force_point = pair_hinge_parent_anchor + np.array(
        [0.4, 0.0, 0.0]
    )
    pair_hinge_child_force_point = pair_hinge_child_anchor - np.array(
        [0.4, 0.0, 0.0]
    )

    max_pair_hinge_broken_anchor_residual = 0.0
    max_pair_hinge_broken_axis_tilt = 0.0
    for _ in range(20):
        restored_pair_hinge_parent.apply_force(
            (-4.0 * pair_hinge_lateral_force).tolist(),
            pair_hinge_parent_force_point.tolist(),
        )
        restored_pair_hinge_child.apply_force(
            (4.0 * pair_hinge_lateral_force).tolist(),
            pair_hinge_child_force_point.tolist(),
        )
        restored_pair_hinge_world.step()
        max_pair_hinge_broken_anchor_residual = max(
            max_pair_hinge_broken_anchor_residual,
            pair_hinge_anchor_residual(),
        )
        max_pair_hinge_broken_axis_tilt = max(
            max_pair_hinge_broken_axis_tilt,
            pair_hinge_axis_tilt(),
        )
    assert max_pair_hinge_broken_anchor_residual > 1e-4
    assert max_pair_hinge_broken_axis_tilt > 1e-4

    restored_pair_hinge_parent.parent_joint.velocity = [0.0] * 6
    restored_pair_hinge_child.parent_joint.velocity = [0.0] * 6
    restored_pair_hinge.command_velocity = [-pair_hinge_speed]
    restored_pair_hinge.break_force = 1.0e12
    restored_pair_hinge.reset_breakage()
    pair_hinge_angle_before_reset = pair_hinge_signed_axis_angle()

    for _ in range(8):
        restored_pair_hinge_world.step()

    assert not restored_pair_hinge.is_broken
    assert restored_pair_hinge.type == sx.JointType.REVOLUTE
    assert restored_pair_hinge.num_dofs == 1
    assert restored_pair_hinge.parent_link == restored_pair_hinge_parent
    assert restored_pair_hinge.child_link == restored_pair_hinge_child
    assert np.asarray(restored_pair_hinge.axis, dtype=float).tolist() == pytest.approx(
        pair_hinge_axis.tolist()
    )
    assert (
        pair_hinge_anchor_residual()
        < max_pair_hinge_broken_anchor_residual * 0.05
    )
    assert pair_hinge_anchor_residual() < 1e-3
    assert pair_hinge_axis_tilt() < max_pair_hinge_broken_axis_tilt * 0.05
    assert pair_hinge_axis_tilt() < 1e-3
    assert (
        pair_hinge_signed_axis_angle()
        < pair_hinge_angle_before_reset - 1e-3
    )

    world_slider_world, _, world_slider_body = _floating_link_world(
        sx, "python_serialized_world_slider_broken"
    )
    world_slider_body.parent_joint.position = [
        0.15,
        -0.05,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    world_slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    world_slider_axis /= np.linalg.norm(world_slider_axis)
    world_slider_child_anchor = np.array([0.2, 0.1, 0.0])
    world_slider_anchor = (
        np.asarray(world_slider_body.translation, dtype=float)
        + np.asarray(world_slider_body.rotation, dtype=float)
        @ world_slider_child_anchor
    )
    world_slider = world_slider_world.add_articulated_prismatic_joint(
        "serialized_world_slider_broken",
        world_slider_body,
        axis=world_slider_axis.tolist(),
        world_anchor=world_slider_anchor.tolist(),
        child_anchor=world_slider_child_anchor.tolist(),
    )
    world_slider.actuator_type = sx.ActuatorType.VELOCITY
    world_slider_speed = 0.3
    world_slider.command_velocity = [world_slider_speed]
    world_slider.set_effort_limits([-1000.0], [1000.0])
    world_slider.break_force = 1e-18

    world_slider_world.enter_simulation_mode()
    world_slider_world.step()
    assert world_slider.is_broken

    world_slider_path = tmp_path / "articulated_world_slider_broken.bin"
    world_slider_world.save_binary(world_slider_path)

    restored_world_slider_world = sx.World()
    restored_world_slider_world.load_binary(world_slider_path)
    restored_world_slider_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_world_slider_arm = restored_world_slider_world.get_multibody(
        "python_serialized_world_slider_broken"
    )
    assert restored_world_slider_arm is not None
    restored_world_slider_body = restored_world_slider_arm.get_link("body")
    restored_world_slider = restored_world_slider_world.get_articulated_joint(
        "serialized_world_slider_broken"
    )
    assert restored_world_slider_body is not None
    assert restored_world_slider is not None
    assert restored_world_slider.is_broken
    assert restored_world_slider.type == sx.JointType.PRISMATIC
    assert restored_world_slider.num_dofs == 1
    assert restored_world_slider.child_link == restored_world_slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_world_slider.parent_link
    assert np.asarray(restored_world_slider.axis, dtype=float).tolist() == pytest.approx(
        world_slider_axis.tolist()
    )
    assert restored_world_slider.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_world_slider.command_velocity.tolist() == pytest.approx(
        [world_slider_speed]
    )
    assert restored_world_slider.effort_lower_limits.tolist() == pytest.approx(
        [-1000.0]
    )
    assert restored_world_slider.effort_upper_limits.tolist() == pytest.approx(
        [1000.0]
    )

    captured_world_slider_rotation = np.asarray(
        restored_world_slider_body.rotation, dtype=float
    ).copy()

    def world_slider_anchor_delta() -> np.ndarray:
        child_anchor_world = (
            np.asarray(restored_world_slider_body.translation, dtype=float)
            + np.asarray(restored_world_slider_body.rotation, dtype=float)
            @ world_slider_child_anchor
        )
        return child_anchor_world - world_slider_anchor

    def world_slider_orthogonal_residual() -> float:
        delta = world_slider_anchor_delta()
        return float(
            np.linalg.norm(
                delta - float(delta @ world_slider_axis) * world_slider_axis
            )
        )

    def world_slider_rotation_error() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_world_slider_body.rotation, dtype=float)
                - captured_world_slider_rotation
            )
        )

    world_slider_lateral_force = np.array([0.0, 1.0, 0.0], dtype=float)
    world_slider_lateral_force -= (
        float(world_slider_lateral_force @ world_slider_axis) * world_slider_axis
    )
    world_slider_lateral_force /= np.linalg.norm(world_slider_lateral_force)
    world_slider_force_point = world_slider_child_anchor + np.array([0.4, 0.0, 0.0])

    max_world_slider_broken_orthogonal_residual = 0.0
    max_world_slider_broken_rotation_error = 0.0
    for _ in range(20):
        restored_world_slider_body.apply_force(
            (4.0 * world_slider_lateral_force).tolist(),
            world_slider_force_point.tolist(),
        )
        restored_world_slider_world.step()
        max_world_slider_broken_orthogonal_residual = max(
            max_world_slider_broken_orthogonal_residual,
            world_slider_orthogonal_residual(),
        )
        max_world_slider_broken_rotation_error = max(
            max_world_slider_broken_rotation_error,
            world_slider_rotation_error(),
        )
    assert max_world_slider_broken_orthogonal_residual > 1e-4
    assert max_world_slider_broken_rotation_error > 1e-4

    restored_world_slider_body.parent_joint.velocity = [0.0] * 6
    restored_world_slider.command_velocity = [-world_slider_speed]
    restored_world_slider.break_force = 1.0e12
    restored_world_slider.reset_breakage()
    world_slider_axis_position_before_reset = float(
        world_slider_anchor_delta() @ world_slider_axis
    )

    for _ in range(8):
        restored_world_slider_world.step()

    assert not restored_world_slider.is_broken
    assert restored_world_slider.type == sx.JointType.PRISMATIC
    assert restored_world_slider.num_dofs == 1
    assert restored_world_slider.child_link == restored_world_slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_world_slider.parent_link
    assert np.asarray(restored_world_slider.axis, dtype=float).tolist() == pytest.approx(
        world_slider_axis.tolist()
    )
    assert (
        world_slider_orthogonal_residual()
        < max_world_slider_broken_orthogonal_residual * 0.05
    )
    assert world_slider_orthogonal_residual() < 1e-3
    assert (
        world_slider_rotation_error()
        < max_world_slider_broken_rotation_error * 0.05
    )
    assert world_slider_rotation_error() < 1e-3
    assert (
        float(world_slider_anchor_delta() @ world_slider_axis)
        < world_slider_axis_position_before_reset - 1e-3
    )


def test_simulation_world_articulated_fixed_spherical_binary_roundtrip_from_python(
    tmp_path: Path,
):
    sx = _simulation()

    fixed_world, fixed_parent, fixed_child = _floating_link_pair_world(
        sx, "python_serialized_pair_fixed"
    )
    fixed_parent_anchor = np.array([0.15, 0.05, 0.0])
    fixed_child_anchor = np.array([-0.15, 0.05, 0.0])
    fixed_child.parent_joint.position = (
        fixed_parent_anchor - fixed_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    fixed_hold = fixed_world.add_articulated_fixed_joint(
        "serialized_pair_fixed",
        fixed_parent,
        fixed_child,
        parent_anchor=fixed_parent_anchor.tolist(),
        child_anchor=fixed_child_anchor.tolist(),
    )
    fixed_hold.break_force = 1e-18
    captured_fixed_relative_rotation = (
        np.asarray(fixed_parent.rotation, dtype=float).T
        @ np.asarray(fixed_child.rotation, dtype=float)
    )

    fixed_world.enter_simulation_mode()
    fixed_parent.apply_force(
        (0.0, -4.0, 0.0),
        (fixed_parent_anchor + np.array([0.4, 0.0, 0.0])).tolist(),
    )
    fixed_child.apply_force(
        (0.0, 4.0, 0.0),
        (fixed_child_anchor - np.array([0.4, 0.0, 0.0])).tolist(),
    )
    fixed_world.step()
    assert fixed_hold.is_broken

    fixed_path = tmp_path / "articulated_pair_fixed.bin"
    fixed_world.save_binary(fixed_path)

    restored_fixed_world = sx.World()
    restored_fixed_world.load_binary(fixed_path)
    restored_fixed_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_fixed_arm = restored_fixed_world.get_multibody(
        "python_serialized_pair_fixed"
    )
    assert restored_fixed_arm is not None
    restored_fixed_parent = restored_fixed_arm.get_link("parent")
    restored_fixed_child = restored_fixed_arm.get_link("child")
    restored_fixed_hold = restored_fixed_world.get_articulated_joint(
        "serialized_pair_fixed"
    )
    assert restored_fixed_parent is not None
    assert restored_fixed_child is not None
    assert restored_fixed_hold is not None
    assert restored_fixed_hold.is_broken
    assert restored_fixed_hold.type == sx.JointType.FIXED
    assert restored_fixed_hold.num_dofs == 0
    assert restored_fixed_hold.parent_link == restored_fixed_parent
    assert restored_fixed_hold.child_link == restored_fixed_child

    def fixed_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(restored_fixed_parent.translation, dtype=float)
            + np.asarray(restored_fixed_parent.rotation, dtype=float)
            @ fixed_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_fixed_child.translation, dtype=float)
            + np.asarray(restored_fixed_child.rotation, dtype=float)
            @ fixed_child_anchor
        )
        return float(np.linalg.norm(parent_anchor_world - child_anchor_world))

    def fixed_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(restored_fixed_parent.rotation, dtype=float).T
            @ np.asarray(restored_fixed_child.rotation, dtype=float)
        )
        return float(
            np.linalg.norm(relative_rotation - captured_fixed_relative_rotation)
        )

    max_fixed_broken_anchor_residual = 0.0
    max_fixed_broken_relative_rotation_error = 0.0
    fixed_parent_force_point = fixed_parent_anchor + np.array([0.4, 0.0, 0.0])
    fixed_child_force_point = fixed_child_anchor - np.array([0.4, 0.0, 0.0])
    for _ in range(20):
        restored_fixed_parent.apply_force(
            (0.0, -4.0, 0.0),
            fixed_parent_force_point.tolist(),
        )
        restored_fixed_child.apply_force(
            (0.0, 4.0, 0.0),
            fixed_child_force_point.tolist(),
        )
        restored_fixed_world.step()
        max_fixed_broken_anchor_residual = max(
            max_fixed_broken_anchor_residual,
            fixed_anchor_residual(),
        )
        max_fixed_broken_relative_rotation_error = max(
            max_fixed_broken_relative_rotation_error,
            fixed_relative_rotation_error(),
        )
    assert max_fixed_broken_anchor_residual > 1e-4
    assert max_fixed_broken_relative_rotation_error > 1e-4

    restored_fixed_parent.parent_joint.velocity = [0.0] * 6
    restored_fixed_child.parent_joint.velocity = [0.0] * 6
    restored_fixed_hold.break_force = 1.0e12
    restored_fixed_hold.reset_breakage()

    for _ in range(8):
        restored_fixed_world.step()

    assert not restored_fixed_hold.is_broken
    assert restored_fixed_hold.type == sx.JointType.FIXED
    assert restored_fixed_hold.num_dofs == 0
    assert restored_fixed_hold.parent_link == restored_fixed_parent
    assert restored_fixed_hold.child_link == restored_fixed_child
    assert fixed_anchor_residual() < max_fixed_broken_anchor_residual * 0.05
    assert fixed_anchor_residual() < 1e-3
    assert (
        fixed_relative_rotation_error()
        < max_fixed_broken_relative_rotation_error * 0.05
    )
    assert fixed_relative_rotation_error() < 1e-3

    socket_world, _, socket_body = _floating_link_world(
        sx, "python_serialized_world_socket"
    )
    socket_body.parent_joint.position = [0.3, -0.2, 0.1, 0.0, 0.0, 0.25]
    socket_child_anchor = np.array([0.2, 0.1, 0.0])
    captured_socket_rotation = np.asarray(socket_body.rotation, dtype=float).copy()
    socket_world_anchor = (
        np.asarray(socket_body.translation, dtype=float)
        + captured_socket_rotation @ socket_child_anchor
    )
    socket = socket_world.add_articulated_spherical_joint(
        "serialized_world_socket",
        socket_body,
        world_anchor=socket_world_anchor.tolist(),
        child_anchor=socket_child_anchor.tolist(),
    )
    socket.break_force = 1e-18

    socket_world.enter_simulation_mode()
    socket_body.apply_force(
        (0.0, 4.0, 0.0),
        (socket_child_anchor + np.array([0.4, 0.0, 0.0])).tolist(),
    )
    socket_world.step()
    assert socket.is_broken

    socket_path = tmp_path / "articulated_world_socket.bin"
    socket_world.save_binary(socket_path)

    restored_socket_world = sx.World()
    restored_socket_world.load_binary(socket_path)
    restored_socket_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_socket_arm = restored_socket_world.get_multibody(
        "python_serialized_world_socket"
    )
    assert restored_socket_arm is not None
    restored_socket_body = restored_socket_arm.get_link("body")
    restored_socket = restored_socket_world.get_articulated_joint(
        "serialized_world_socket"
    )
    assert restored_socket_body is not None
    assert restored_socket is not None
    assert restored_socket.is_broken
    assert restored_socket.type == sx.JointType.SPHERICAL
    assert restored_socket.num_dofs == 3
    assert restored_socket.child_link == restored_socket_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_socket.parent_link

    def socket_anchor_residual() -> float:
        socket_anchor_world = (
            np.asarray(restored_socket_body.translation, dtype=float)
            + np.asarray(restored_socket_body.rotation, dtype=float)
            @ socket_child_anchor
        )
        return float(np.linalg.norm(socket_anchor_world - socket_world_anchor))

    def socket_rotation_error() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_socket_body.rotation, dtype=float)
                - captured_socket_rotation
            )
        )

    max_socket_broken_anchor_residual = 0.0
    max_socket_broken_rotation_error = 0.0
    socket_force_point = socket_child_anchor + np.array([0.4, 0.0, 0.0])
    for _ in range(20):
        restored_socket_body.apply_force(
            (0.0, 4.0, 0.0),
            socket_force_point.tolist(),
        )
        restored_socket_world.step()
        max_socket_broken_anchor_residual = max(
            max_socket_broken_anchor_residual,
            socket_anchor_residual(),
        )
        max_socket_broken_rotation_error = max(
            max_socket_broken_rotation_error,
            socket_rotation_error(),
        )
    assert max_socket_broken_anchor_residual > 1e-4
    assert max_socket_broken_rotation_error > 1e-4

    restored_socket_body.parent_joint.velocity = [0.0] * 6
    restored_socket.break_force = 1.0e12
    restored_socket.reset_breakage()

    for _ in range(8):
        restored_socket_world.step()

    assert not restored_socket.is_broken
    assert restored_socket.type == sx.JointType.SPHERICAL
    assert restored_socket.num_dofs == 3
    assert restored_socket.child_link == restored_socket_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_socket.parent_link
    assert socket_anchor_residual() < max_socket_broken_anchor_residual * 0.05
    assert socket_anchor_residual() < 1e-3
    assert socket_rotation_error() > max_socket_broken_rotation_error * 0.5


def test_simulation_world_articulated_fixed_spherical_binary_roundtrip_from_python_completes_endpoint_types(
    tmp_path: Path,
):
    sx = _simulation()

    fixed_world, _, fixed_body = _floating_link_world(
        sx, "python_serialized_world_fixed"
    )
    fixed_body.parent_joint.position = [0.25, -0.15, 0.1, 0.0, 0.0, 0.3]
    fixed_child_anchor = np.array([0.2, 0.1, 0.0])
    captured_fixed_rotation = np.asarray(fixed_body.rotation, dtype=float).copy()
    fixed_world_anchor = (
        np.asarray(fixed_body.translation, dtype=float)
        + captured_fixed_rotation @ fixed_child_anchor
    )
    fixed_hold = fixed_world.add_articulated_fixed_joint(
        "serialized_world_fixed",
        fixed_body,
        world_anchor=fixed_world_anchor.tolist(),
        child_anchor=fixed_child_anchor.tolist(),
    )
    fixed_hold.break_force = 1e-18

    fixed_world.enter_simulation_mode()
    fixed_body.apply_force(
        (0.0, 4.0, 0.0),
        (fixed_child_anchor + np.array([0.4, 0.0, 0.0])).tolist(),
    )
    fixed_world.step()
    assert fixed_hold.is_broken

    fixed_path = tmp_path / "articulated_world_fixed.bin"
    fixed_world.save_binary(fixed_path)

    restored_fixed_world = sx.World()
    restored_fixed_world.load_binary(fixed_path)
    restored_fixed_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_fixed_arm = restored_fixed_world.get_multibody(
        "python_serialized_world_fixed"
    )
    assert restored_fixed_arm is not None
    restored_fixed_body = restored_fixed_arm.get_link("body")
    restored_fixed_hold = restored_fixed_world.get_articulated_joint(
        "serialized_world_fixed"
    )
    assert restored_fixed_body is not None
    assert restored_fixed_hold is not None
    assert restored_fixed_hold.is_broken
    assert restored_fixed_hold.type == sx.JointType.FIXED
    assert restored_fixed_hold.num_dofs == 0
    assert restored_fixed_hold.child_link == restored_fixed_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_fixed_hold.parent_link

    def fixed_anchor_residual() -> float:
        anchor_world = (
            np.asarray(restored_fixed_body.translation, dtype=float)
            + np.asarray(restored_fixed_body.rotation, dtype=float)
            @ fixed_child_anchor
        )
        return float(np.linalg.norm(anchor_world - fixed_world_anchor))

    def fixed_rotation_error() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_fixed_body.rotation, dtype=float)
                - captured_fixed_rotation
            )
        )

    max_fixed_broken_anchor_residual = 0.0
    max_fixed_broken_rotation_error = 0.0
    fixed_force_point = fixed_child_anchor + np.array([0.4, 0.0, 0.0])
    for _ in range(20):
        restored_fixed_body.apply_force(
            (0.0, 4.0, 0.0),
            fixed_force_point.tolist(),
        )
        restored_fixed_world.step()
        max_fixed_broken_anchor_residual = max(
            max_fixed_broken_anchor_residual,
            fixed_anchor_residual(),
        )
        max_fixed_broken_rotation_error = max(
            max_fixed_broken_rotation_error,
            fixed_rotation_error(),
        )
    assert max_fixed_broken_anchor_residual > 1e-4
    assert max_fixed_broken_rotation_error > 1e-4

    restored_fixed_body.parent_joint.velocity = [0.0] * 6
    restored_fixed_hold.break_force = 1.0e12
    restored_fixed_hold.reset_breakage()

    for _ in range(8):
        restored_fixed_world.step()

    assert not restored_fixed_hold.is_broken
    assert restored_fixed_hold.type == sx.JointType.FIXED
    assert restored_fixed_hold.num_dofs == 0
    assert restored_fixed_hold.child_link == restored_fixed_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_fixed_hold.parent_link
    assert fixed_anchor_residual() < max_fixed_broken_anchor_residual * 0.05
    assert fixed_anchor_residual() < 1e-3
    assert fixed_rotation_error() < max_fixed_broken_rotation_error * 0.05
    assert fixed_rotation_error() < 1e-3

    socket_world, socket_parent, socket_child = _floating_link_pair_world(
        sx, "python_serialized_pair_socket"
    )
    socket_parent_anchor = np.array([0.15, 0.05, 0.0])
    socket_child_anchor = np.array([-0.15, 0.05, 0.0])
    socket_child.parent_joint.position = (
        socket_parent_anchor - socket_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    captured_socket_relative_rotation = (
        np.asarray(socket_parent.rotation, dtype=float).T
        @ np.asarray(socket_child.rotation, dtype=float)
    )
    socket = socket_world.add_articulated_spherical_joint(
        "serialized_pair_socket",
        socket_parent,
        socket_child,
        parent_anchor=socket_parent_anchor.tolist(),
        child_anchor=socket_child_anchor.tolist(),
    )
    socket.break_force = 1e-18

    socket_world.enter_simulation_mode()
    socket_parent.apply_force(
        (0.0, -4.0, 0.0),
        (socket_parent_anchor + np.array([0.4, 0.0, 0.0])).tolist(),
    )
    socket_child.apply_force(
        (0.0, 4.0, 0.0),
        (socket_child_anchor - np.array([0.4, 0.0, 0.0])).tolist(),
    )
    socket_world.step()
    assert socket.is_broken

    socket_path = tmp_path / "articulated_pair_socket.bin"
    socket_world.save_binary(socket_path)

    restored_socket_world = sx.World()
    restored_socket_world.load_binary(socket_path)
    restored_socket_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    restored_socket_arm = restored_socket_world.get_multibody(
        "python_serialized_pair_socket"
    )
    assert restored_socket_arm is not None
    restored_socket_parent = restored_socket_arm.get_link("parent")
    restored_socket_child = restored_socket_arm.get_link("child")
    restored_socket = restored_socket_world.get_articulated_joint(
        "serialized_pair_socket"
    )
    assert restored_socket_parent is not None
    assert restored_socket_child is not None
    assert restored_socket is not None
    assert restored_socket.is_broken
    assert restored_socket.type == sx.JointType.SPHERICAL
    assert restored_socket.num_dofs == 3
    assert restored_socket.parent_link == restored_socket_parent
    assert restored_socket.child_link == restored_socket_child

    def socket_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(restored_socket_parent.translation, dtype=float)
            + np.asarray(restored_socket_parent.rotation, dtype=float)
            @ socket_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_socket_child.translation, dtype=float)
            + np.asarray(restored_socket_child.rotation, dtype=float)
            @ socket_child_anchor
        )
        return float(np.linalg.norm(parent_anchor_world - child_anchor_world))

    def socket_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(restored_socket_parent.rotation, dtype=float).T
            @ np.asarray(restored_socket_child.rotation, dtype=float)
        )
        return float(
            np.linalg.norm(relative_rotation - captured_socket_relative_rotation)
        )

    max_socket_broken_anchor_residual = 0.0
    max_socket_broken_relative_rotation_error = 0.0
    socket_parent_force_point = socket_parent_anchor + np.array([0.4, 0.0, 0.0])
    socket_child_force_point = socket_child_anchor - np.array([0.4, 0.0, 0.0])
    for _ in range(20):
        restored_socket_parent.apply_force(
            (0.0, -4.0, 0.0),
            socket_parent_force_point.tolist(),
        )
        restored_socket_child.apply_force(
            (0.0, 4.0, 0.0),
            socket_child_force_point.tolist(),
        )
        restored_socket_world.step()
        max_socket_broken_anchor_residual = max(
            max_socket_broken_anchor_residual,
            socket_anchor_residual(),
        )
        max_socket_broken_relative_rotation_error = max(
            max_socket_broken_relative_rotation_error,
            socket_relative_rotation_error(),
        )
    assert max_socket_broken_anchor_residual > 1e-4
    assert max_socket_broken_relative_rotation_error > 1e-4

    restored_socket_parent.parent_joint.velocity = [0.0] * 6
    restored_socket_child.parent_joint.velocity = [0.0] * 6
    restored_socket.break_force = 1.0e12
    restored_socket.reset_breakage()

    for _ in range(8):
        restored_socket_world.step()

    assert not restored_socket.is_broken
    assert restored_socket.type == sx.JointType.SPHERICAL
    assert restored_socket.num_dofs == 3
    assert restored_socket.parent_link == restored_socket_parent
    assert restored_socket.child_link == restored_socket_child
    assert socket_anchor_residual() < max_socket_broken_anchor_residual * 0.05
    assert socket_anchor_residual() < 1e-3
    assert (
        socket_relative_rotation_error()
        > max_socket_broken_relative_rotation_error * 0.5
    )


def test_simulation_world_articulated_design_binary_rebuild_from_python(
    tmp_path: Path,
):
    sx = _simulation()

    fixed_world, fixed_parent, fixed_child = _floating_link_pair_world(
        sx, "python_serialized_design_pair_fixed"
    )
    fixed_parent_anchor = np.array([0.2, 0.1, 0.0])
    fixed_child_anchor = np.array([-0.1, 0.1, 0.0])
    fixed_child.parent_joint.position = (
        fixed_parent_anchor - fixed_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    fixed_world.add_articulated_fixed_joint(
        "serialized_design_pair_fixed",
        fixed_parent,
        fixed_child,
        parent_anchor=fixed_parent_anchor.tolist(),
        child_anchor=fixed_child_anchor.tolist(),
    )

    fixed_path = tmp_path / "articulated_design_pair_fixed.bin"
    fixed_world.save_binary(fixed_path)

    restored_fixed_world = sx.World()
    restored_fixed_world.load_binary(fixed_path)
    restored_fixed_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_fixed_world.is_simulation_mode
    restored_fixed_arm = restored_fixed_world.get_multibody(
        "python_serialized_design_pair_fixed"
    )
    assert restored_fixed_arm is not None
    restored_fixed_parent = restored_fixed_arm.get_link("parent")
    restored_fixed_child = restored_fixed_arm.get_link("child")
    restored_fixed_hold = restored_fixed_world.get_articulated_joint(
        "serialized_design_pair_fixed"
    )
    assert restored_fixed_parent is not None
    assert restored_fixed_child is not None
    assert restored_fixed_hold is not None
    assert not restored_fixed_hold.is_broken
    assert restored_fixed_hold.type == sx.JointType.FIXED
    assert restored_fixed_hold.num_dofs == 0
    assert restored_fixed_hold.parent_link == restored_fixed_parent
    assert restored_fixed_hold.child_link == restored_fixed_child

    restored_fixed_world.enter_simulation_mode()
    captured_fixed_relative_rotation = (
        np.asarray(restored_fixed_parent.rotation, dtype=float).T
        @ np.asarray(restored_fixed_child.rotation, dtype=float)
    )

    def fixed_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(restored_fixed_parent.translation, dtype=float)
            + np.asarray(restored_fixed_parent.rotation, dtype=float)
            @ fixed_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_fixed_child.translation, dtype=float)
            + np.asarray(restored_fixed_child.rotation, dtype=float)
            @ fixed_child_anchor
        )
        return float(np.linalg.norm(parent_anchor_world - child_anchor_world))

    def fixed_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(restored_fixed_parent.rotation, dtype=float).T
            @ np.asarray(restored_fixed_child.rotation, dtype=float)
        )
        return float(
            np.linalg.norm(relative_rotation - captured_fixed_relative_rotation)
        )

    max_fixed_anchor_residual = 0.0
    max_fixed_relative_rotation_error = 0.0
    fixed_parent_force_point = fixed_parent_anchor + np.array([0.4, 0.0, 0.0])
    fixed_child_force_point = fixed_child_anchor - np.array([0.4, 0.0, 0.0])
    for _ in range(30):
        restored_fixed_parent.apply_force(
            (0.0, -4.0, 0.0),
            fixed_parent_force_point.tolist(),
        )
        restored_fixed_child.apply_force(
            (0.0, 4.0, 0.0),
            fixed_child_force_point.tolist(),
        )
        restored_fixed_world.step()
        max_fixed_anchor_residual = max(
            max_fixed_anchor_residual,
            fixed_anchor_residual(),
        )
        max_fixed_relative_rotation_error = max(
            max_fixed_relative_rotation_error,
            fixed_relative_rotation_error(),
        )

    assert not restored_fixed_hold.is_broken
    assert max_fixed_anchor_residual < 1e-5
    assert max_fixed_relative_rotation_error < 1e-5

    slider_world, slider_parent, slider_child = _floating_link_pair_world(
        sx, "python_serialized_design_pair_slider"
    )
    slider_parent_anchor = np.array([0.2, 0.1, 0.0])
    slider_child_anchor = np.array([-0.1, 0.1, 0.0])
    slider_child.parent_joint.position = (
        slider_parent_anchor - slider_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    slider_axis /= np.linalg.norm(slider_axis)
    slider = slider_world.add_articulated_prismatic_joint(
        "serialized_design_pair_slider",
        slider_parent,
        slider_child,
        axis=slider_axis.tolist(),
        parent_anchor=slider_parent_anchor.tolist(),
        child_anchor=slider_child_anchor.tolist(),
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1000.0], [1000.0])

    slider_path = tmp_path / "articulated_design_pair_slider.bin"
    slider_world.save_binary(slider_path)

    restored_slider_world = sx.World()
    restored_slider_world.load_binary(slider_path)
    restored_slider_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_slider_world.is_simulation_mode
    restored_slider_arm = restored_slider_world.get_multibody(
        "python_serialized_design_pair_slider"
    )
    assert restored_slider_arm is not None
    restored_slider_parent = restored_slider_arm.get_link("parent")
    restored_slider_child = restored_slider_arm.get_link("child")
    restored_slider = restored_slider_world.get_articulated_joint(
        "serialized_design_pair_slider"
    )
    assert restored_slider_parent is not None
    assert restored_slider_child is not None
    assert restored_slider is not None
    assert not restored_slider.is_broken
    assert restored_slider.type == sx.JointType.PRISMATIC
    assert restored_slider.num_dofs == 1
    assert restored_slider.parent_link == restored_slider_parent
    assert restored_slider.child_link == restored_slider_child
    assert np.asarray(restored_slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    assert restored_slider.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_slider.command_velocity.tolist() == pytest.approx([slider_speed])
    assert restored_slider.effort_lower_limits.tolist() == pytest.approx([-1000.0])
    assert restored_slider.effort_upper_limits.tolist() == pytest.approx([1000.0])

    restored_slider_world.enter_simulation_mode()
    captured_slider_relative_rotation = (
        np.asarray(restored_slider_parent.rotation, dtype=float).T
        @ np.asarray(restored_slider_child.rotation, dtype=float)
    )

    def slider_anchor_delta() -> np.ndarray:
        parent_anchor_world = (
            np.asarray(restored_slider_parent.translation, dtype=float)
            + np.asarray(restored_slider_parent.rotation, dtype=float)
            @ slider_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_slider_child.translation, dtype=float)
            + np.asarray(restored_slider_child.rotation, dtype=float)
            @ slider_child_anchor
        )
        return child_anchor_world - parent_anchor_world

    def slider_orthogonal_residual() -> float:
        delta = slider_anchor_delta()
        return float(
            np.linalg.norm(delta - float(delta @ slider_axis) * slider_axis)
        )

    def slider_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(restored_slider_parent.rotation, dtype=float).T
            @ np.asarray(restored_slider_child.rotation, dtype=float)
        )
        return float(
            np.linalg.norm(relative_rotation - captured_slider_relative_rotation)
        )

    slider_axis_position_before = float(slider_anchor_delta() @ slider_axis)
    max_slider_orthogonal_residual = 0.0
    max_slider_relative_rotation_error = 0.0
    for _ in range(12):
        restored_slider_world.step()
        max_slider_orthogonal_residual = max(
            max_slider_orthogonal_residual,
            slider_orthogonal_residual(),
        )
        max_slider_relative_rotation_error = max(
            max_slider_relative_rotation_error,
            slider_relative_rotation_error(),
        )

    assert not restored_slider.is_broken
    assert max_slider_orthogonal_residual < 1e-3
    assert max_slider_relative_rotation_error < 1e-6
    assert (
        float(slider_anchor_delta() @ slider_axis)
        > slider_axis_position_before + 1e-3
    )

    hinge_world, hinge_parent, hinge_child = _floating_link_pair_world(
        sx, "python_serialized_design_pair_hinge"
    )
    hinge_parent_anchor = np.array([0.2, 0.1, 0.0])
    hinge_child_anchor = np.array([-0.1, 0.1, 0.0])
    hinge_child.parent_joint.position = (
        hinge_parent_anchor - hinge_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    hinge_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    hinge_axis /= np.linalg.norm(hinge_axis)
    hinge = hinge_world.add_articulated_revolute_joint(
        "serialized_design_pair_hinge",
        hinge_parent,
        hinge_child,
        axis=hinge_axis.tolist(),
        parent_anchor=hinge_parent_anchor.tolist(),
        child_anchor=hinge_child_anchor.tolist(),
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])

    hinge_path = tmp_path / "articulated_design_pair_hinge.bin"
    hinge_world.save_binary(hinge_path)

    restored_hinge_world = sx.World()
    restored_hinge_world.load_binary(hinge_path)
    restored_hinge_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_hinge_world.is_simulation_mode
    restored_hinge_arm = restored_hinge_world.get_multibody(
        "python_serialized_design_pair_hinge"
    )
    assert restored_hinge_arm is not None
    restored_hinge_parent = restored_hinge_arm.get_link("parent")
    restored_hinge_child = restored_hinge_arm.get_link("child")
    restored_hinge = restored_hinge_world.get_articulated_joint(
        "serialized_design_pair_hinge"
    )
    assert restored_hinge_parent is not None
    assert restored_hinge_child is not None
    assert restored_hinge is not None
    assert not restored_hinge.is_broken
    assert restored_hinge.type == sx.JointType.REVOLUTE
    assert restored_hinge.num_dofs == 1
    assert restored_hinge.parent_link == restored_hinge_parent
    assert restored_hinge.child_link == restored_hinge_child
    assert np.asarray(restored_hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    assert restored_hinge.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_hinge.command_velocity.tolist() == pytest.approx([hinge_speed])
    assert restored_hinge.effort_lower_limits.tolist() == pytest.approx([-1000.0])
    assert restored_hinge.effort_upper_limits.tolist() == pytest.approx([1000.0])

    restored_hinge_world.enter_simulation_mode()

    def hinge_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(restored_hinge_parent.translation, dtype=float)
            + np.asarray(restored_hinge_parent.rotation, dtype=float)
            @ hinge_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_hinge_child.translation, dtype=float)
            + np.asarray(restored_hinge_child.rotation, dtype=float)
            @ hinge_child_anchor
        )
        return float(np.linalg.norm(child_anchor_world - parent_anchor_world))

    def hinge_relative_rotation() -> np.ndarray:
        return (
            np.asarray(restored_hinge_parent.rotation, dtype=float).T
            @ np.asarray(restored_hinge_child.rotation, dtype=float)
        )

    def hinge_signed_axis_angle() -> float:
        rotation = hinge_relative_rotation()
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_hinge_parent.rotation, dtype=float) @ hinge_axis
                - np.asarray(restored_hinge_child.rotation, dtype=float) @ hinge_axis
            )
        )

    hinge_angle_before = hinge_signed_axis_angle()
    max_hinge_anchor_residual = 0.0
    max_hinge_axis_tilt = 0.0
    for _ in range(12):
        restored_hinge_world.step()
        max_hinge_anchor_residual = max(
            max_hinge_anchor_residual,
            hinge_anchor_residual(),
        )
        max_hinge_axis_tilt = max(max_hinge_axis_tilt, hinge_axis_tilt())

    assert not restored_hinge.is_broken
    assert max_hinge_anchor_residual < 1e-6
    assert max_hinge_axis_tilt < 1e-3
    assert hinge_signed_axis_angle() > hinge_angle_before + 1e-3

    world_slider_world, _, world_slider_body = _floating_link_world(
        sx, "python_serialized_design_world_slider"
    )
    world_slider_body.parent_joint.position = [
        0.15,
        -0.05,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    world_slider_axis = np.array([1.0, 2.0, 0.5], dtype=float)
    world_slider_axis /= np.linalg.norm(world_slider_axis)
    world_slider_child_anchor = np.array([0.2, 0.1, 0.0])
    world_slider_anchor = (
        np.asarray(world_slider_body.translation, dtype=float)
        + np.asarray(world_slider_body.rotation, dtype=float)
        @ world_slider_child_anchor
    )
    world_slider = world_slider_world.add_articulated_prismatic_joint(
        "serialized_design_world_slider",
        world_slider_body,
        axis=world_slider_axis.tolist(),
        world_anchor=world_slider_anchor.tolist(),
        child_anchor=world_slider_child_anchor.tolist(),
    )
    world_slider.actuator_type = sx.ActuatorType.VELOCITY
    world_slider_speed = 0.3
    world_slider.command_velocity = [world_slider_speed]
    world_slider.set_effort_limits([-1000.0], [1000.0])

    world_slider_path = tmp_path / "articulated_design_world_slider.bin"
    world_slider_world.save_binary(world_slider_path)

    restored_world_slider_world = sx.World()
    restored_world_slider_world.load_binary(world_slider_path)
    restored_world_slider_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_world_slider_world.is_simulation_mode
    restored_world_slider_arm = restored_world_slider_world.get_multibody(
        "python_serialized_design_world_slider"
    )
    assert restored_world_slider_arm is not None
    restored_world_slider_body = restored_world_slider_arm.get_link("body")
    restored_world_slider = restored_world_slider_world.get_articulated_joint(
        "serialized_design_world_slider"
    )
    assert restored_world_slider_body is not None
    assert restored_world_slider is not None
    assert not restored_world_slider.is_broken
    assert restored_world_slider.type == sx.JointType.PRISMATIC
    assert restored_world_slider.num_dofs == 1
    assert restored_world_slider.child_link == restored_world_slider_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_world_slider.parent_link
    assert np.asarray(restored_world_slider.axis, dtype=float).tolist() == pytest.approx(
        world_slider_axis.tolist()
    )
    assert restored_world_slider.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_world_slider.command_velocity.tolist() == pytest.approx(
        [world_slider_speed]
    )
    assert restored_world_slider.effort_lower_limits.tolist() == pytest.approx(
        [-1000.0]
    )
    assert restored_world_slider.effort_upper_limits.tolist() == pytest.approx(
        [1000.0]
    )

    restored_world_slider_world.enter_simulation_mode()
    captured_world_slider_rotation = np.asarray(
        restored_world_slider_body.rotation, dtype=float
    ).copy()

    def world_slider_anchor_delta() -> np.ndarray:
        child_anchor_world = (
            np.asarray(restored_world_slider_body.translation, dtype=float)
            + np.asarray(restored_world_slider_body.rotation, dtype=float)
            @ world_slider_child_anchor
        )
        return child_anchor_world - world_slider_anchor

    def world_slider_orthogonal_residual() -> float:
        delta = world_slider_anchor_delta()
        return float(
            np.linalg.norm(
                delta - float(delta @ world_slider_axis) * world_slider_axis
            )
        )

    def world_slider_rotation_error() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_world_slider_body.rotation, dtype=float)
                - captured_world_slider_rotation
            )
        )

    world_slider_axis_position_before = float(
        world_slider_anchor_delta() @ world_slider_axis
    )
    max_world_slider_orthogonal_residual = 0.0
    max_world_slider_rotation_error = 0.0
    for _ in range(12):
        restored_world_slider_world.step()
        max_world_slider_orthogonal_residual = max(
            max_world_slider_orthogonal_residual,
            world_slider_orthogonal_residual(),
        )
        max_world_slider_rotation_error = max(
            max_world_slider_rotation_error,
            world_slider_rotation_error(),
        )

    assert not restored_world_slider.is_broken
    assert max_world_slider_orthogonal_residual < 1e-3
    assert max_world_slider_rotation_error < 1e-6
    assert (
        float(world_slider_anchor_delta() @ world_slider_axis)
        > world_slider_axis_position_before + 1e-3
    )

    world_hinge_world, _, world_hinge_body = _floating_link_world(
        sx, "python_serialized_design_world_hinge"
    )
    world_hinge_body.parent_joint.position = [
        0.15,
        -0.05,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    world_hinge_axis = np.array([1.0, 2.0, 3.0], dtype=float)
    world_hinge_axis /= np.linalg.norm(world_hinge_axis)
    world_hinge_child_anchor = np.array([0.2, 0.1, 0.0])
    world_hinge_anchor = (
        np.asarray(world_hinge_body.translation, dtype=float)
        + np.asarray(world_hinge_body.rotation, dtype=float)
        @ world_hinge_child_anchor
    )
    world_hinge = world_hinge_world.add_articulated_revolute_joint(
        "serialized_design_world_hinge",
        world_hinge_body,
        axis=world_hinge_axis.tolist(),
        world_anchor=world_hinge_anchor.tolist(),
        child_anchor=world_hinge_child_anchor.tolist(),
    )
    world_hinge.actuator_type = sx.ActuatorType.VELOCITY
    world_hinge_speed = 0.4
    world_hinge.command_velocity = [world_hinge_speed]
    world_hinge.set_effort_limits([-1000.0], [1000.0])

    world_hinge_path = tmp_path / "articulated_design_world_hinge.bin"
    world_hinge_world.save_binary(world_hinge_path)

    restored_world_hinge_world = sx.World()
    restored_world_hinge_world.load_binary(world_hinge_path)
    restored_world_hinge_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_world_hinge_world.is_simulation_mode
    restored_world_hinge_arm = restored_world_hinge_world.get_multibody(
        "python_serialized_design_world_hinge"
    )
    assert restored_world_hinge_arm is not None
    restored_world_hinge_body = restored_world_hinge_arm.get_link("body")
    restored_world_hinge = restored_world_hinge_world.get_articulated_joint(
        "serialized_design_world_hinge"
    )
    assert restored_world_hinge_body is not None
    assert restored_world_hinge is not None
    assert not restored_world_hinge.is_broken
    assert restored_world_hinge.type == sx.JointType.REVOLUTE
    assert restored_world_hinge.num_dofs == 1
    assert restored_world_hinge.child_link == restored_world_hinge_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_world_hinge.parent_link
    assert np.asarray(restored_world_hinge.axis, dtype=float).tolist() == pytest.approx(
        world_hinge_axis.tolist()
    )
    assert restored_world_hinge.actuator_type == sx.ActuatorType.VELOCITY
    assert restored_world_hinge.command_velocity.tolist() == pytest.approx(
        [world_hinge_speed]
    )
    assert restored_world_hinge.effort_lower_limits.tolist() == pytest.approx(
        [-1000.0]
    )
    assert restored_world_hinge.effort_upper_limits.tolist() == pytest.approx(
        [1000.0]
    )

    restored_world_hinge_world.enter_simulation_mode()

    def world_hinge_anchor_residual() -> float:
        child_anchor_world = (
            np.asarray(restored_world_hinge_body.translation, dtype=float)
            + np.asarray(restored_world_hinge_body.rotation, dtype=float)
            @ world_hinge_child_anchor
        )
        return float(np.linalg.norm(child_anchor_world - world_hinge_anchor))

    def world_hinge_signed_axis_angle() -> float:
        rotation = np.asarray(restored_world_hinge_body.rotation, dtype=float)
        skew_axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        sin_term = 0.5 * float(world_hinge_axis @ skew_axis)
        cos_term = 0.5 * (float(np.trace(rotation)) - 1.0)
        return math.atan2(sin_term, cos_term)

    def world_hinge_axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_world_hinge_body.rotation, dtype=float)
                @ world_hinge_axis
                - world_hinge_axis
            )
        )

    world_hinge_angle_before = world_hinge_signed_axis_angle()
    max_world_hinge_anchor_residual = 0.0
    max_world_hinge_axis_tilt = 0.0
    for _ in range(12):
        restored_world_hinge_world.step()
        max_world_hinge_anchor_residual = max(
            max_world_hinge_anchor_residual,
            world_hinge_anchor_residual(),
        )
        max_world_hinge_axis_tilt = max(
            max_world_hinge_axis_tilt,
            world_hinge_axis_tilt(),
        )

    assert not restored_world_hinge.is_broken
    assert max_world_hinge_anchor_residual < 1e-6
    assert max_world_hinge_axis_tilt < 1e-3
    assert world_hinge_signed_axis_angle() > world_hinge_angle_before + 1e-3

    socket_world, _, socket_body = _floating_link_world(
        sx, "python_serialized_design_world_socket"
    )
    socket_body.parent_joint.position = [0.3, -0.2, 0.1, 0.0, 0.0, 0.25]
    socket_child_anchor = np.array([0.2, 0.1, 0.0])
    socket_world_anchor = (
        np.asarray(socket_body.translation, dtype=float)
        + np.asarray(socket_body.rotation, dtype=float) @ socket_child_anchor
    )
    socket_world.add_articulated_spherical_joint(
        "serialized_design_world_socket",
        socket_body,
        world_anchor=socket_world_anchor.tolist(),
        child_anchor=socket_child_anchor.tolist(),
    )

    socket_path = tmp_path / "articulated_design_world_socket.bin"
    socket_world.save_binary(socket_path)

    restored_socket_world = sx.World()
    restored_socket_world.load_binary(socket_path)
    restored_socket_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_socket_world.is_simulation_mode
    restored_socket_arm = restored_socket_world.get_multibody(
        "python_serialized_design_world_socket"
    )
    assert restored_socket_arm is not None
    restored_socket_body = restored_socket_arm.get_link("body")
    restored_socket = restored_socket_world.get_articulated_joint(
        "serialized_design_world_socket"
    )
    assert restored_socket_body is not None
    assert restored_socket is not None
    assert not restored_socket.is_broken
    assert restored_socket.type == sx.JointType.SPHERICAL
    assert restored_socket.num_dofs == 3
    assert restored_socket.child_link == restored_socket_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_socket.parent_link

    restored_socket_world.enter_simulation_mode()
    captured_socket_rotation = np.asarray(
        restored_socket_body.rotation, dtype=float
    ).copy()

    def socket_anchor_residual() -> float:
        socket_anchor_world = (
            np.asarray(restored_socket_body.translation, dtype=float)
            + np.asarray(restored_socket_body.rotation, dtype=float)
            @ socket_child_anchor
        )
        return float(np.linalg.norm(socket_anchor_world - socket_world_anchor))

    def socket_rotation_error() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_socket_body.rotation, dtype=float)
                - captured_socket_rotation
            )
        )

    max_socket_anchor_residual = 0.0
    max_socket_rotation_error = 0.0
    socket_force_point = socket_child_anchor + np.array([0.4, 0.0, 0.0])
    for _ in range(40):
        restored_socket_body.apply_force(
            (0.0, 4.0, 0.0),
            socket_force_point.tolist(),
        )
        restored_socket_world.step()
        max_socket_anchor_residual = max(
            max_socket_anchor_residual,
            socket_anchor_residual(),
        )
        max_socket_rotation_error = max(
            max_socket_rotation_error,
            socket_rotation_error(),
        )

    assert not restored_socket.is_broken
    assert max_socket_anchor_residual < 1e-5
    assert max_socket_rotation_error > 1e-4


def test_simulation_world_articulated_design_binary_rebuild_from_python_completes_fixed_spherical_endpoint_types(
    tmp_path: Path,
):
    sx = _simulation()

    fixed_world, _, fixed_body = _floating_link_world(
        sx, "python_serialized_design_world_fixed"
    )
    fixed_body.parent_joint.position = [0.25, -0.15, 0.1, 0.0, 0.0, 0.3]
    fixed_child_anchor = np.array([0.2, 0.1, 0.0])
    fixed_rotation = np.asarray(fixed_body.rotation, dtype=float).copy()
    fixed_world_anchor = (
        np.asarray(fixed_body.translation, dtype=float)
        + fixed_rotation @ fixed_child_anchor
    )
    fixed_world.add_articulated_fixed_joint(
        "serialized_design_world_fixed",
        fixed_body,
        world_anchor=fixed_world_anchor.tolist(),
        child_anchor=fixed_child_anchor.tolist(),
    )

    fixed_path = tmp_path / "articulated_design_world_fixed.bin"
    fixed_world.save_binary(fixed_path)

    restored_fixed_world = sx.World()
    restored_fixed_world.load_binary(fixed_path)
    restored_fixed_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_fixed_world.is_simulation_mode
    restored_fixed_arm = restored_fixed_world.get_multibody(
        "python_serialized_design_world_fixed"
    )
    assert restored_fixed_arm is not None
    restored_fixed_body = restored_fixed_arm.get_link("body")
    restored_fixed_hold = restored_fixed_world.get_articulated_joint(
        "serialized_design_world_fixed"
    )
    assert restored_fixed_body is not None
    assert restored_fixed_hold is not None
    assert not restored_fixed_hold.is_broken
    assert restored_fixed_hold.type == sx.JointType.FIXED
    assert restored_fixed_hold.num_dofs == 0
    assert restored_fixed_hold.child_link == restored_fixed_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = restored_fixed_hold.parent_link

    restored_fixed_world.enter_simulation_mode()

    def fixed_anchor_residual() -> float:
        anchor_world = (
            np.asarray(restored_fixed_body.translation, dtype=float)
            + np.asarray(restored_fixed_body.rotation, dtype=float)
            @ fixed_child_anchor
        )
        return float(np.linalg.norm(anchor_world - fixed_world_anchor))

    def fixed_rotation_error() -> float:
        return float(
            np.linalg.norm(
                np.asarray(restored_fixed_body.rotation, dtype=float)
                - fixed_rotation
            )
        )

    max_fixed_anchor_residual = 0.0
    max_fixed_rotation_error = 0.0
    fixed_force_point = fixed_child_anchor + np.array([0.4, 0.0, 0.0])
    for _ in range(30):
        restored_fixed_body.apply_force(
            (0.0, 4.0, 0.0),
            fixed_force_point.tolist(),
        )
        restored_fixed_world.step()
        max_fixed_anchor_residual = max(
            max_fixed_anchor_residual,
            fixed_anchor_residual(),
        )
        max_fixed_rotation_error = max(
            max_fixed_rotation_error,
            fixed_rotation_error(),
        )

    assert not restored_fixed_hold.is_broken
    assert max_fixed_anchor_residual < 1e-5
    assert max_fixed_rotation_error < 1e-5

    socket_world, socket_parent, socket_child = _floating_link_pair_world(
        sx, "python_serialized_design_pair_socket"
    )
    socket_parent_anchor = np.array([0.15, 0.05, 0.0])
    socket_child_anchor = np.array([-0.15, 0.05, 0.0])
    socket_child.parent_joint.position = (
        socket_parent_anchor - socket_child_anchor
    ).tolist() + [0.0, 0.0, 0.0]
    captured_socket_relative_rotation = (
        np.asarray(socket_parent.rotation, dtype=float).T
        @ np.asarray(socket_child.rotation, dtype=float)
    )
    socket_world.add_articulated_spherical_joint(
        "serialized_design_pair_socket",
        socket_parent,
        socket_child,
        parent_anchor=socket_parent_anchor.tolist(),
        child_anchor=socket_child_anchor.tolist(),
    )

    socket_path = tmp_path / "articulated_design_pair_socket.bin"
    socket_world.save_binary(socket_path)

    restored_socket_world = sx.World()
    restored_socket_world.load_binary(socket_path)
    restored_socket_world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    assert not restored_socket_world.is_simulation_mode
    restored_socket_arm = restored_socket_world.get_multibody(
        "python_serialized_design_pair_socket"
    )
    assert restored_socket_arm is not None
    restored_socket_parent = restored_socket_arm.get_link("parent")
    restored_socket_child = restored_socket_arm.get_link("child")
    restored_socket = restored_socket_world.get_articulated_joint(
        "serialized_design_pair_socket"
    )
    assert restored_socket_parent is not None
    assert restored_socket_child is not None
    assert restored_socket is not None
    assert not restored_socket.is_broken
    assert restored_socket.type == sx.JointType.SPHERICAL
    assert restored_socket.num_dofs == 3
    assert restored_socket.parent_link == restored_socket_parent
    assert restored_socket.child_link == restored_socket_child

    restored_socket_world.enter_simulation_mode()

    def socket_anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(restored_socket_parent.translation, dtype=float)
            + np.asarray(restored_socket_parent.rotation, dtype=float)
            @ socket_parent_anchor
        )
        child_anchor_world = (
            np.asarray(restored_socket_child.translation, dtype=float)
            + np.asarray(restored_socket_child.rotation, dtype=float)
            @ socket_child_anchor
        )
        return float(np.linalg.norm(parent_anchor_world - child_anchor_world))

    def socket_relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(restored_socket_parent.rotation, dtype=float).T
            @ np.asarray(restored_socket_child.rotation, dtype=float)
        )
        return float(
            np.linalg.norm(relative_rotation - captured_socket_relative_rotation)
        )

    max_socket_anchor_residual = 0.0
    max_socket_relative_rotation_error = 0.0
    socket_parent_force_point = socket_parent_anchor + np.array([0.4, 0.0, 0.0])
    socket_child_force_point = socket_child_anchor - np.array([0.4, 0.0, 0.0])
    for _ in range(30):
        restored_socket_parent.apply_force(
            (0.0, -4.0, 0.0),
            socket_parent_force_point.tolist(),
        )
        restored_socket_child.apply_force(
            (0.0, 4.0, 0.0),
            socket_child_force_point.tolist(),
        )
        restored_socket_world.step()
        max_socket_anchor_residual = max(
            max_socket_anchor_residual,
            socket_anchor_residual(),
        )
        max_socket_relative_rotation_error = max(
            max_socket_relative_rotation_error,
            socket_relative_rotation_error(),
        )

    assert not restored_socket.is_broken
    assert max_socket_anchor_residual < 1e-3
    assert max_socket_relative_rotation_error > 1e-4


def test_simulation_world_articulated_pair_motor_breakage_reset_from_python():
    sx = _simulation()

    world, parent, child = _floating_link_pair_world(
        sx, "python_resettable_pair_breakable_slider"
    )
    child.parent_joint.position = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]

    slider_axis = np.array([1.0, 0.0, 0.0])
    parent_anchor = np.array([0.2, 0.1, 0.0])
    child_anchor = np.array([-0.1, 0.1, 0.0])
    slider = world.add_articulated_prismatic_joint(
        "resettable_pair_slider",
        parent,
        child,
        axis=slider_axis,
        parent_anchor=parent_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.parent_link == parent
    assert slider.child_link == child
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    slider.actuator_type = sx.ActuatorType.VELOCITY
    slider_speed = 0.3
    slider.command_velocity = [slider_speed]
    slider.set_effort_limits([-1000.0], [1000.0])
    slider.break_force = 1e-18

    def anchor_delta() -> np.ndarray:
        parent_anchor_world = (
            np.asarray(parent.translation, dtype=float)
            + np.asarray(parent.rotation, dtype=float) @ parent_anchor
        )
        child_anchor_world = (
            np.asarray(child.translation, dtype=float)
            + np.asarray(child.rotation, dtype=float) @ child_anchor
        )
        return child_anchor_world - parent_anchor_world

    def orthogonal_anchor_residual() -> float:
        delta = anchor_delta()
        orthogonal = delta - delta.dot(slider_axis) * slider_axis
        return float(np.linalg.norm(orthogonal))

    def relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(parent.rotation, dtype=float).T
            @ np.asarray(child.rotation, dtype=float)
        )
        return float(np.linalg.norm(relative_rotation - np.eye(3)))

    world.enter_simulation_mode()
    world.step()

    assert slider.is_broken
    first_step_delta = anchor_delta()
    assert float(first_step_delta.dot(slider_axis)) == pytest.approx(
        slider_speed * world.time_step, abs=1e-6
    )
    assert orthogonal_anchor_residual() < 1e-6
    assert relative_rotation_error() < 1e-6

    max_broken_orthogonal_residual = 0.0
    for _ in range(20):
        parent.apply_force((0.0, -4.0, 0.0), parent_anchor.tolist())
        child.apply_force((0.0, 4.0, 0.0), child_anchor.tolist())
        world.step()
        max_broken_orthogonal_residual = max(
            max_broken_orthogonal_residual, orthogonal_anchor_residual()
        )
    assert max_broken_orthogonal_residual > 1e-4

    parent.parent_joint.velocity = [0.0] * 6
    child.parent_joint.velocity = [0.0] * 6
    slider.break_force = 1.0e12
    slider.reset_breakage()

    assert not slider.is_broken

    world.step()

    assert not slider.is_broken
    assert slider.type == sx.JointType.PRISMATIC
    assert slider.num_dofs == 1
    assert slider.parent_link == parent
    assert slider.child_link == child
    assert np.asarray(slider.axis, dtype=float).tolist() == pytest.approx(
        slider_axis.tolist()
    )
    reset_orthogonal_residual = orthogonal_anchor_residual()
    assert reset_orthogonal_residual < max_broken_orthogonal_residual * 0.05
    assert reset_orthogonal_residual < 1e-3
    assert relative_rotation_error() < 1e-6
    assert float(anchor_delta().dot(slider_axis)) > float(
        first_step_delta.dot(slider_axis)
    )


def test_simulation_world_articulated_pair_revolute_breakage_reset_from_python():
    sx = _simulation()

    world, parent, child = _floating_link_pair_world(
        sx, "python_resettable_pair_breakable_hinge"
    )
    child.parent_joint.position = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]

    hinge_axis = np.array([0.0, 0.0, 1.0])
    parent_anchor = np.array([0.2, 0.1, 0.0])
    child_anchor = np.array([-0.1, 0.1, 0.0])
    hinge = world.add_articulated_revolute_joint(
        "resettable_pair_hinge",
        parent,
        child,
        axis=hinge_axis,
        parent_anchor=parent_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == parent
    assert hinge.child_link == child
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    hinge.actuator_type = sx.ActuatorType.VELOCITY
    hinge_speed = 0.4
    hinge.command_velocity = [hinge_speed]
    hinge.set_effort_limits([-1000.0], [1000.0])
    hinge.break_force = 1e-18

    def anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(parent.translation, dtype=float)
            + np.asarray(parent.rotation, dtype=float) @ parent_anchor
        )
        child_anchor_world = (
            np.asarray(child.translation, dtype=float)
            + np.asarray(child.rotation, dtype=float) @ child_anchor
        )
        return float(
            np.linalg.norm(
                child_anchor_world - parent_anchor_world
            )
        )

    def axis_tilt() -> float:
        return float(
            np.linalg.norm(
                np.asarray(parent.rotation, dtype=float) @ hinge_axis
                - np.asarray(child.rotation, dtype=float) @ hinge_axis
            )
        )

    def relative_yaw() -> float:
        relative_rotation = (
            np.asarray(parent.rotation, dtype=float).T
            @ np.asarray(child.rotation, dtype=float)
        )
        return math.atan2(relative_rotation[1, 0], relative_rotation[0, 0])

    world.enter_simulation_mode()
    world.step()

    assert hinge.is_broken
    first_step_yaw = relative_yaw()
    assert first_step_yaw == pytest.approx(hinge_speed * world.time_step, abs=1e-6)
    assert anchor_residual() < 1e-6
    assert axis_tilt() < 1e-6

    max_broken_anchor_residual = 0.0
    for _ in range(20):
        parent.apply_force(
            (0.0, -4.0, 0.0),
            (parent_anchor + np.array([0.3, 0.0, 0.0])).tolist(),
        )
        child.apply_force(
            (0.0, 4.0, 0.0),
            (child_anchor - np.array([0.3, 0.0, 0.0])).tolist(),
        )
        world.step()
        max_broken_anchor_residual = max(
            max_broken_anchor_residual, anchor_residual()
        )
    assert max_broken_anchor_residual > 1e-4

    parent.parent_joint.velocity = [0.0] * 6
    child.parent_joint.velocity = [0.0] * 6
    hinge.break_force = 1.0e12
    hinge.reset_breakage()

    assert not hinge.is_broken

    world.step()

    assert not hinge.is_broken
    assert hinge.type == sx.JointType.REVOLUTE
    assert hinge.num_dofs == 1
    assert hinge.parent_link == parent
    assert hinge.child_link == child
    assert np.asarray(hinge.axis, dtype=float).tolist() == pytest.approx(
        hinge_axis.tolist()
    )
    assert anchor_residual() < 1e-6
    assert axis_tilt() < 1e-6
    assert relative_yaw() > first_step_yaw


def test_simulation_world_articulated_fixed_breakage_reset_reengages_from_python():
    sx = _simulation()

    same_world, same_base, same_body = _floating_link_world(
        sx, "python_resettable_breakable_fixed"
    )
    same_body.parent_joint.position = [0.1, -0.2, 0.15, 0.0, 0.0, 0.2]
    same_child_anchor = np.array([-0.15, 0.05, 0.0])
    same_parent_anchor = (
        np.asarray(same_body.translation, dtype=float)
        + np.asarray(same_body.rotation, dtype=float) @ same_child_anchor
    )
    same_hold = same_world.add_articulated_fixed_joint(
        "resettable_hold",
        same_base,
        same_body,
        parent_anchor=same_parent_anchor.tolist(),
        child_anchor=same_child_anchor.tolist(),
    )
    assert same_hold.type == sx.JointType.FIXED
    assert same_hold.num_dofs == 0
    assert same_hold.parent_link == same_base
    assert same_hold.child_link == same_body
    same_hold.break_force = 1e-18
    same_off_center_point = (same_child_anchor + np.array([0.4, 0.0, 0.0])).tolist()

    same_world.enter_simulation_mode()
    captured_translation = np.asarray(same_body.translation, dtype=float).copy()
    captured_rotation = np.asarray(same_body.rotation, dtype=float).copy()

    same_body.apply_force((0.0, 4.0, 0.0), same_off_center_point)
    same_world.step()

    assert same_hold.is_broken

    max_broken_translation_error = 0.0
    max_broken_anchor_residual = 0.0
    for _ in range(20):
        same_body.apply_force((0.0, 4.0, 0.0), same_off_center_point)
        same_world.step()
        anchor_position = (
            np.asarray(same_body.translation, dtype=float)
            + np.asarray(same_body.rotation, dtype=float) @ same_child_anchor
        )
        max_broken_translation_error = max(
            max_broken_translation_error,
            float(
                np.linalg.norm(
                    np.asarray(same_body.translation, dtype=float)
                    - captured_translation
                )
            ),
        )
        max_broken_anchor_residual = max(
            max_broken_anchor_residual,
            float(np.linalg.norm(anchor_position - same_parent_anchor)),
        )
    assert max_broken_translation_error > 1e-4
    assert max_broken_anchor_residual > 1e-4

    same_body.parent_joint.velocity = [0.0] * 6
    same_hold.break_force = 1.0e12
    same_hold.reset_breakage()

    assert not same_hold.is_broken

    same_world.step()

    assert not same_hold.is_broken
    assert same_hold.type == sx.JointType.FIXED
    assert same_hold.num_dofs == 0
    assert same_hold.parent_link == same_base
    assert same_hold.child_link == same_body
    assert (
        np.linalg.norm(
            np.asarray(same_body.translation, dtype=float) - captured_translation
        )
        < 1e-6
    )
    assert (
        np.linalg.norm(np.asarray(same_body.rotation, dtype=float) - captured_rotation)
        < 1e-6
    )
    assert (
        np.linalg.norm(
            np.asarray(same_body.translation, dtype=float)
            + np.asarray(same_body.rotation, dtype=float) @ same_child_anchor
            - same_parent_anchor
        )
        < 1e-6
    )

    world_anchor_world, _, world_anchor_body = _floating_link_world(
        sx, "python_resettable_world_breakable_fixed"
    )
    world_anchor_body.parent_joint.position = [0.3, -0.2, 0.1, 0.0, 0.0, 0.25]
    world_child_anchor = np.array([0.2, 0.1, 0.0])
    captured_world_rotation = np.asarray(world_anchor_body.rotation, dtype=float).copy()
    world_anchor = (
        np.asarray(world_anchor_body.translation, dtype=float)
        + captured_world_rotation @ world_child_anchor
    )
    world_hold = world_anchor_world.add_articulated_fixed_joint(
        "resettable_world_hold",
        world_anchor_body,
        world_anchor=world_anchor.tolist(),
        child_anchor=world_child_anchor.tolist(),
    )
    assert world_hold.type == sx.JointType.FIXED
    assert world_hold.num_dofs == 0
    assert world_hold.child_link == world_anchor_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_hold.parent_link
    world_hold.break_force = 1e-18
    off_center_point = (world_child_anchor + np.array([0.4, 0.0, 0.0])).tolist()

    world_anchor_world.enter_simulation_mode()
    captured_world_translation = np.asarray(
        world_anchor_body.translation, dtype=float
    ).copy()

    world_anchor_body.apply_force((0.0, 4.0, 0.0), off_center_point)
    world_anchor_world.step()

    assert world_hold.is_broken

    max_broken_world_translation_error = 0.0
    max_broken_world_rotation_error = 0.0
    max_broken_world_anchor_residual = 0.0
    for _ in range(20):
        world_anchor_body.apply_force((0.0, 4.0, 0.0), off_center_point)
        world_anchor_world.step()
        anchor_position = (
            np.asarray(world_anchor_body.translation, dtype=float)
            + np.asarray(world_anchor_body.rotation, dtype=float) @ world_child_anchor
        )
        max_broken_world_translation_error = max(
            max_broken_world_translation_error,
            float(
                np.linalg.norm(
                    np.asarray(world_anchor_body.translation, dtype=float)
                    - captured_world_translation
                )
            ),
        )
        max_broken_world_rotation_error = max(
            max_broken_world_rotation_error,
            float(
                np.linalg.norm(
                    np.asarray(world_anchor_body.rotation, dtype=float)
                    - captured_world_rotation
                )
            ),
        )
        max_broken_world_anchor_residual = max(
            max_broken_world_anchor_residual,
            float(np.linalg.norm(anchor_position - world_anchor)),
        )
    assert max_broken_world_translation_error > 1e-4
    assert max_broken_world_rotation_error > 1e-4
    assert max_broken_world_anchor_residual > 1e-4

    world_anchor_body.parent_joint.velocity = [0.0] * 6
    world_hold.break_force = 1.0e12
    world_hold.reset_breakage()

    assert not world_hold.is_broken

    world_anchor_world.step()

    assert not world_hold.is_broken
    assert world_hold.type == sx.JointType.FIXED
    assert world_hold.num_dofs == 0
    assert world_hold.child_link == world_anchor_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_hold.parent_link
    assert (
        np.linalg.norm(
            np.asarray(world_anchor_body.translation, dtype=float)
            - captured_world_translation
        )
        < 1e-6
    )
    assert (
        np.linalg.norm(
            np.asarray(world_anchor_body.translation, dtype=float)
            + np.asarray(world_anchor_body.rotation, dtype=float) @ world_child_anchor
            - world_anchor
        )
        < 1e-6
    )
    assert (
        np.linalg.norm(
            np.asarray(world_anchor_body.rotation, dtype=float)
            - captured_world_rotation
        )
        < 1e-6
    )


def test_simulation_world_articulated_fixed_pair_breakage_reset_from_python():
    sx = _simulation()

    world, parent, child = _floating_link_pair_world(
        sx, "python_resettable_pair_breakable_fixed"
    )
    child.parent_joint.position = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]

    parent_anchor = np.array([0.15, 0.05, 0.0])
    child_anchor = np.array([-0.15, 0.05, 0.0])
    hold = world.add_articulated_fixed_joint(
        "resettable_pair_hold",
        parent,
        child,
        parent_anchor=parent_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert hold.type == sx.JointType.FIXED
    assert hold.num_dofs == 0
    assert hold.parent_link == parent
    assert hold.child_link == child
    hold.break_force = 1e-18
    parent_force_point = (parent_anchor + np.array([0.4, 0.0, 0.0])).tolist()
    child_force_point = (child_anchor - np.array([0.4, 0.0, 0.0])).tolist()

    world.enter_simulation_mode()
    captured_parent_rotation = np.asarray(parent.rotation, dtype=float).copy()
    captured_child_rotation = np.asarray(child.rotation, dtype=float).copy()
    captured_relative_rotation = captured_parent_rotation.T @ captured_child_rotation

    parent.apply_force((0.0, -4.0, 0.0), parent_force_point)
    child.apply_force((0.0, 4.0, 0.0), child_force_point)
    world.step()

    assert hold.is_broken

    def anchor_residual() -> float:
        parent_anchor_world = (
            np.asarray(parent.translation, dtype=float)
            + np.asarray(parent.rotation, dtype=float) @ parent_anchor
        )
        child_anchor_world = (
            np.asarray(child.translation, dtype=float)
            + np.asarray(child.rotation, dtype=float) @ child_anchor
        )
        return float(np.linalg.norm(parent_anchor_world - child_anchor_world))

    def relative_rotation_error() -> float:
        relative_rotation = (
            np.asarray(parent.rotation, dtype=float).T
            @ np.asarray(child.rotation, dtype=float)
        )
        return float(np.linalg.norm(relative_rotation - captured_relative_rotation))

    max_broken_anchor_residual = 0.0
    max_broken_relative_rotation_error = 0.0
    for _ in range(20):
        parent.apply_force((0.0, -4.0, 0.0), parent_force_point)
        child.apply_force((0.0, 4.0, 0.0), child_force_point)
        world.step()
        max_broken_anchor_residual = max(
            max_broken_anchor_residual, anchor_residual()
        )
        max_broken_relative_rotation_error = max(
            max_broken_relative_rotation_error, relative_rotation_error()
        )
    assert max_broken_anchor_residual > 1e-4
    assert max_broken_relative_rotation_error > 1e-4

    parent.parent_joint.velocity = [0.0] * 6
    child.parent_joint.velocity = [0.0] * 6
    hold.break_force = 1.0e12
    hold.reset_breakage()

    assert not hold.is_broken

    world.step()

    assert not hold.is_broken
    assert hold.type == sx.JointType.FIXED
    assert hold.num_dofs == 0
    assert hold.parent_link == parent
    assert hold.child_link == child
    assert anchor_residual() < 1e-6
    assert relative_rotation_error() < 1e-6


def test_simulation_world_articulated_spherical_pair_breakage_reset_reengages_from_python():
    sx = _simulation()

    same_world, same_base, same_body = _floating_link_world(
        sx, "python_resettable_breakable_socket"
    )
    same_body.parent_joint.position = [0.1, -0.2, 0.15, 0.0, 0.0, 0.2]
    child_anchor = np.array([-0.15, 0.05, 0.0])
    parent_anchor = (
        np.asarray(same_body.translation, dtype=float)
        + np.asarray(same_body.rotation, dtype=float) @ child_anchor
    )
    same_socket = same_world.add_articulated_spherical_joint(
        "resettable_socket",
        same_base,
        same_body,
        parent_anchor=parent_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert same_socket.type == sx.JointType.SPHERICAL
    assert same_socket.num_dofs == 3
    assert same_socket.parent_link == same_base
    assert same_socket.child_link == same_body
    same_socket.break_force = 1e-18
    off_center_point = (child_anchor + np.array([0.4, 0.0, 0.0])).tolist()

    same_world.enter_simulation_mode()
    captured_rotation = np.asarray(same_body.rotation, dtype=float).copy()

    same_body.apply_force((0.0, 4.0, 0.0), off_center_point)
    same_world.step()

    assert same_socket.is_broken

    max_broken_translation_error = 0.0
    max_broken_rotation_error = 0.0
    for _ in range(20):
        same_body.apply_force((0.0, 4.0, 0.0), off_center_point)
        same_world.step()
        anchor_position = (
            np.asarray(same_body.translation, dtype=float)
            + np.asarray(same_body.rotation, dtype=float) @ child_anchor
        )
        max_broken_translation_error = max(
            max_broken_translation_error,
            float(np.linalg.norm(anchor_position - parent_anchor)),
        )
        max_broken_rotation_error = max(
            max_broken_rotation_error,
            float(
                np.linalg.norm(
                    np.asarray(same_body.rotation, dtype=float) - captured_rotation
                )
            ),
        )
    assert max_broken_translation_error > 1e-4
    assert max_broken_rotation_error > 1e-4

    same_body.parent_joint.velocity = [0.0] * 6
    same_socket.break_force = 1.0e12
    same_socket.reset_breakage()

    assert not same_socket.is_broken

    same_world.step()

    assert not same_socket.is_broken
    assert same_socket.type == sx.JointType.SPHERICAL
    assert same_socket.num_dofs == 3
    assert same_socket.parent_link == same_base
    assert same_socket.child_link == same_body
    assert (
        np.linalg.norm(
            np.asarray(same_body.translation, dtype=float)
            + np.asarray(same_body.rotation, dtype=float) @ child_anchor
            - parent_anchor
        )
        < 1e-6
    )
    assert (
        np.linalg.norm(np.asarray(same_body.rotation, dtype=float) - captured_rotation)
        > 1e-4
    )


def test_simulation_world_articulated_spherical_breakage_steps_from_python():
    sx = _simulation()

    same_world, same_base, same_body = _floating_link_world(
        sx, "python_breakable_socket"
    )
    same_socket = same_world.add_articulated_spherical_joint(
        "breakable_socket", same_base, same_body
    )
    assert same_socket.type == sx.JointType.SPHERICAL
    assert same_socket.num_dofs == 3
    assert same_socket.parent_link == same_base
    assert same_socket.child_link == same_body
    same_socket.break_force = 1e-18

    same_world.enter_simulation_mode()
    captured_position = np.asarray(same_body.translation, dtype=float).copy()
    same_body.apply_force((0.0, 4.0, 0.0))
    same_world.step()

    assert same_socket.is_broken

    max_broken_translation_error = 0.0
    for _ in range(20):
        same_body.apply_force((0.0, 4.0, 0.0))
        same_world.step()
        max_broken_translation_error = max(
            max_broken_translation_error,
            float(
                np.linalg.norm(
                    np.asarray(same_body.translation, dtype=float) - captured_position
                )
            ),
        )
    assert max_broken_translation_error > 1e-4

    world_anchor_world, _, world_anchor_body = _floating_link_world(
        sx, "python_world_breakable_socket"
    )
    world_anchor_body.parent_joint.position = [0.3, -0.2, 0.1, 0.0, 0.0, 0.25]
    child_anchor = np.array([0.2, 0.1, 0.0])
    captured_rotation = np.asarray(world_anchor_body.rotation, dtype=float).copy()
    world_anchor = (
        np.asarray(world_anchor_body.translation, dtype=float)
        + captured_rotation @ child_anchor
    )
    world_socket = world_anchor_world.add_articulated_spherical_joint(
        "world_breakable_socket",
        world_anchor_body,
        world_anchor=world_anchor.tolist(),
        child_anchor=child_anchor.tolist(),
    )
    assert world_socket.type == sx.JointType.SPHERICAL
    assert world_socket.num_dofs == 3
    assert world_socket.child_link == world_anchor_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_socket.parent_link
    world_socket.break_force = 1e-18
    off_center_point = (child_anchor + np.array([0.4, 0.0, 0.0])).tolist()

    world_anchor_world.enter_simulation_mode()
    world_anchor_body.apply_force((0.0, 4.0, 0.0), off_center_point)
    world_anchor_world.step()

    assert world_socket.is_broken

    max_broken_anchor_residual = 0.0
    max_broken_rotation_error = 0.0
    for _ in range(20):
        world_anchor_body.apply_force((0.0, 4.0, 0.0), off_center_point)
        world_anchor_world.step()
        anchor_position = (
            np.asarray(world_anchor_body.translation, dtype=float)
            + np.asarray(world_anchor_body.rotation, dtype=float) @ child_anchor
        )
        max_broken_anchor_residual = max(
            max_broken_anchor_residual,
            float(np.linalg.norm(anchor_position - world_anchor)),
        )
        max_broken_rotation_error = max(
            max_broken_rotation_error,
            float(
                np.linalg.norm(
                    np.asarray(world_anchor_body.rotation, dtype=float)
                    - captured_rotation
                )
            ),
        )
    assert max_broken_anchor_residual > 1e-4
    assert max_broken_rotation_error > 1e-4

    world_anchor_body.parent_joint.velocity = [0.0] * 6
    world_socket.break_force = 1.0e12
    world_socket.reset_breakage()

    assert not world_socket.is_broken

    world_anchor_world.step()

    reset_anchor_position = (
        np.asarray(world_anchor_body.translation, dtype=float)
        + np.asarray(world_anchor_body.rotation, dtype=float) @ child_anchor
    )
    assert not world_socket.is_broken
    assert world_socket.type == sx.JointType.SPHERICAL
    assert world_socket.num_dofs == 3
    assert world_socket.child_link == world_anchor_body
    with pytest.raises(Exception, match="parent endpoint"):
        _ = world_socket.parent_link
    assert np.linalg.norm(reset_anchor_position - world_anchor) < 1e-6
    assert (
        np.linalg.norm(
            np.asarray(world_anchor_body.rotation, dtype=float) - captured_rotation
        )
        > 1e-4
    )


def test_simulation_multibody_link_joint_common_path():
    sx = _simulation()

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
    sx = _simulation()

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


def test_simulation_add_skeleton_imports_legacy_revolute_tree():
    sx = _simulation()

    skeleton = dart.Skeleton("py_loader")
    properties = dart.RevoluteJointProperties()
    properties.mAxis = np.asarray([0.0, 1.0, 0.0], dtype=float)
    joint, body = skeleton.create_revolute_joint_and_body_node_pair(None, properties)
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


def test_simulation_add_skeleton_loads_uri():
    sx = _simulation()

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


def test_simulation_add_skeleton_uri_accepts_read_options():
    sx = _simulation()

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


def test_simulation_api_does_not_expose_add_world_bridge():
    sx = _simulation()
    repo_root = Path(__file__).resolve().parents[4]
    stub = (repo_root / "python" / "stubs" / "dartpy" / "simulation.pyi").read_text(
        encoding="utf-8"
    )

    assert not hasattr(sx, "add_world")
    assert '"add_world"' not in stub
    assert "def add_world(" not in stub


def test_simulation_loop_closure_topology_api():
    sx = _simulation()

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


def test_simulation_frame_handles_support_kinematics_only_updates():
    sx = _simulation()

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


def test_simulation_world_common_path_properties_and_step_count():
    sx = _simulation()

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


def test_simulation_rigid_body_options_value_object():
    sx = _simulation()

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


def test_simulation_world_gravity():
    sx = _simulation()

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


def test_simulation_multibody_options_selector():
    sx = _simulation()

    configured = sx.World(
        rigid_body_solver=sx.RigidBodySolver.IPC,
        multibody_options=sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL,
            variational_max_iterations=200,
            variational_tolerance=1.0e-9,
        ),
    )
    assert configured.rigid_body_solver == sx.RigidBodySolver.IPC
    assert configured.multibody_options.integration_family == sx.MultibodyIntegrationFamily.VARIATIONAL
    assert configured.multibody_options.variational_max_iterations == 200
    assert configured.multibody_options.variational_tolerance == pytest.approx(1.0e-9)

    policy_configured = sx.World(
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
        contact_gradient_mode=sx.ContactGradientMode.PRE_CONTACT_SURROGATE,
        compute_accelerator_policy=sx.ComputeAcceleratorPolicy.PREFER_ACCELERATED,
    )
    assert policy_configured.contact_solver_method == sx.ContactSolverMethod.BOXED_LCP
    assert (
        policy_configured.contact_gradient_mode
        == sx.ContactGradientMode.PRE_CONTACT_SURROGATE
    )
    assert (
        policy_configured.compute_accelerator_policy
        == sx.ComputeAcceleratorPolicy.PREFER_ACCELERATED
    )
    policy_configured.contact_gradient_mode = sx.ContactGradientMode.ANALYTIC
    assert policy_configured.contact_gradient_mode == sx.ContactGradientMode.ANALYTIC
    policy_configured.compute_accelerator_policy = sx.ComputeAcceleratorPolicy.CPU_ONLY
    assert (
        policy_configured.compute_accelerator_policy
        == sx.ComputeAcceleratorPolicy.CPU_ONLY
    )
    with pytest.raises(TypeError):
        policy_configured.contact_gradient_mode = 99
    with pytest.raises(TypeError):
        policy_configured.compute_accelerator_policy = 99

    with pytest.raises(Exception):
        sx.World(multibody_options=sx.MultibodyOptions(integration_family="nonsense"))

    world = sx.World()
    # Multibody solver config is a value object; selection is by the typed
    # method-family enum (MultibodyIntegrationFamily).
    assert world.multibody_options.integration_family == sx.MultibodyIntegrationFamily.SEMI_IMPLICIT
    assert world.multibody_options.variational_max_iterations == 100
    assert world.multibody_options.variational_tolerance == pytest.approx(1.0e-10)
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL,
        variational_max_iterations=120,
        variational_tolerance=2.0e-9,
    )
    assert world.multibody_options.integration_family == sx.MultibodyIntegrationFamily.VARIATIONAL
    assert world.multibody_options.variational_max_iterations == 120
    assert world.multibody_options.variational_tolerance == pytest.approx(2.0e-9)
    with pytest.raises(Exception):
        world.multibody_options = sx.MultibodyOptions(integration_family="nonsense")
    # A rejected assignment leaves the previous valid selection in place.
    assert world.multibody_options.integration_family == sx.MultibodyIntegrationFamily.VARIATIONAL
    assert world.multibody_options.variational_max_iterations == 120
    with pytest.raises(Exception):
        world.multibody_options = sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL,
            variational_max_iterations=0,
        )
    with pytest.raises(Exception):
        world.multibody_options = sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL,
            variational_tolerance=0.0,
        )

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


def test_simulation_rigid_body_dynamic_quantities():
    sx = _simulation()

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


def test_simulation_rigid_body_impulses_update_momentum():
    sx = _simulation()

    world = sx.World(gravity=(0.0, 0.0, 0.0))
    body = world.add_rigid_body("body", mass=2.0)
    body.inertia = ((2.0, 0.0, 0.0), (0.0, 4.0, 0.0), (0.0, 0.0, 3.0))

    body.apply_linear_impulse((4.0, -2.0, 0.0))
    body.apply_angular_impulse((0.0, 8.0, 6.0))

    assert body.linear_velocity.tolist() == pytest.approx([2.0, -1.0, 0.0])
    assert body.linear_momentum.tolist() == pytest.approx([4.0, -2.0, 0.0])
    assert body.angular_velocity.tolist() == pytest.approx([0.0, 2.0, 2.0])
    assert body.angular_momentum.tolist() == pytest.approx([0.0, 8.0, 6.0])

    static_body = world.add_rigid_body("static_body")
    static_body.is_static = True
    static_body.apply_linear_impulse((4.0, 0.0, 0.0))
    static_body.apply_angular_impulse((0.0, 0.0, 4.0))
    assert static_body.linear_velocity.tolist() == pytest.approx([0.0, 0.0, 0.0])
    assert static_body.angular_velocity.tolist() == pytest.approx([0.0, 0.0, 0.0])


def test_simulation_multibody_forward_dynamics():
    sx = _simulation()

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


def test_simulation_screw_joint_dynamics():
    sx = _simulation()

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


def test_simulation_universal_joint_dynamics():
    sx = _simulation()

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


def test_simulation_planar_joint_dynamics():
    sx = _simulation()

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


def test_simulation_ball_joint_dynamics():
    sx = _simulation()

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


def test_simulation_free_joint_dynamics():
    sx = _simulation()

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


def test_simulation_link_center_of_mass_offset():
    sx = _simulation()

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


def test_simulation_multibody_dynamics_terms():
    sx = _simulation()

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


def test_simulation_multibody_equation_of_motion_consistency():
    sx = _simulation()

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


def test_simulation_multibody_inverse_dynamics():
    sx = _simulation()

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


def test_simulation_multibody_impulse_response():
    sx = _simulation()

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


def test_simulation_multibody_link_jacobian():
    sx = _simulation()

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


def test_simulation_multibody_link_world_jacobian():
    sx = _simulation()

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


def test_simulation_multibody_dynamics_terms_no_dof():
    sx = _simulation()

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


def test_simulation_joint_armature():
    sx = _simulation()

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


def test_simulation_joint_coulomb_friction():
    sx = _simulation()

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


def test_simulation_joint_actuator_types():
    sx = _simulation()

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


def test_simulation_joint_velocity_actuator():
    sx = _simulation()

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


def test_simulation_joint_spring_and_damping():
    sx = _simulation()

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


def test_simulation_joint_position_limit():
    sx = _simulation()

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


def test_simulation_joint_effort_limit():
    sx = _simulation()

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


def test_simulation_joint_velocity_limit():
    sx = _simulation()

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


def test_simulation_collision_query():
    sx = _simulation()

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


def test_simulation_collision_shape_local_transform():
    sx = _simulation()

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


def test_simulation_compound_collision_shapes():
    sx = _simulation()

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
        sx.CollisionShape.box((0.1, 0.2, 0.3), _translation_transform(1.0, 0.0, 0.0))
    )
    assert base.has_collision_shape
    assert len(base.collision_shapes) == 2
    assert base.collision_shape.type == sx.CollisionShapeType.SPHERE


def test_simulation_collision_shape_capsule():
    sx = _simulation()

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


def test_simulation_collision_shape_cylinder():
    sx = _simulation()

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


def test_simulation_collision_shape_plane():
    sx = _simulation()

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


def test_simulation_collision_shape_mesh():
    sx = _simulation()

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


def test_simulation_mesh_collision_shape_public_surface():
    sx = _simulation()

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


def test_simulation_mesh_collision_shape_drives_rigid_ipc_body():
    sx = _simulation()

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


def test_simulation_cylinder_collision_shape_public_surface_and_query():
    sx = _simulation()

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


def test_simulation_kinematic_body():
    sx = _simulation()

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


def test_simulation_static_setter_clears_kinematic_body():
    sx = _simulation()

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


def test_simulation_collision_query_includes_links():
    sx = _simulation()

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


def test_simulation_collision_query_can_filter_same_multibody_link_pairs():
    sx = _simulation()

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


def test_simulation_collision_query_can_filter_body_type_pairs():
    sx = _simulation()

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


def test_simulation_collision_query_can_ignore_specific_pairs():
    sx = _simulation()

    def has_contact_between(contacts, first: str, second: str) -> bool:
        return any(
            {contact.body_a.name, contact.body_b.name} == {first, second}
            for contact in contacts
        )

    world = sx.World()
    body_a = world.add_rigid_body("rigid_a")
    body_a.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body_b = world.add_rigid_body("rigid_b", position=(0.4, 0.0, 0.0))
    body_b.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body_c = world.add_rigid_body("rigid_c", position=(0.8, 0.0, 0.0))
    body_c.set_collision_shape(sx.CollisionShape.sphere(0.5))

    assert has_contact_between(world.collide(), "rigid_a", "rigid_b")
    assert world.num_ignored_collision_pairs == 0

    world.set_collision_pair_ignored(body_a, body_b)
    assert world.is_collision_pair_ignored(body_b, body_a)
    assert world.num_ignored_collision_pairs == 1

    filtered_contacts = world.collide()
    assert not has_contact_between(filtered_contacts, "rigid_a", "rigid_b")
    assert has_contact_between(
        filtered_contacts, "rigid_a", "rigid_c"
    ) or has_contact_between(filtered_contacts, "rigid_b", "rigid_c")

    world.set_collision_pair_ignored(body_b, body_a, False)
    assert not world.is_collision_pair_ignored(body_a, body_b)
    assert world.num_ignored_collision_pairs == 0
    assert has_contact_between(world.collide(), "rigid_a", "rigid_b")

    world.set_collision_pair_ignored(body_a, body_b)
    world.clear_ignored_collision_pairs()
    assert world.num_ignored_collision_pairs == 0
    assert has_contact_between(world.collide(), "rigid_a", "rigid_b")

    other_world = sx.World()
    foreign_body = other_world.add_rigid_body("foreign")
    with pytest.raises(Exception, match="same frame|distinct"):
        world.set_collision_pair_ignored(body_a, body_a)
    with pytest.raises(Exception, match="different world"):
        world.set_collision_pair_ignored(body_a, foreign_body)

    link_world = sx.World()
    robot = link_world.add_multibody("robot")
    link = robot.add_link("link")
    link.set_collision_shape(sx.CollisionShape.sphere(0.5))
    body = link_world.add_rigid_body("rigid", position=(0.4, 0.0, 0.0))
    body.set_collision_shape(sx.CollisionShape.sphere(0.5))

    link_world.enter_simulation_mode()
    assert has_contact_between(link_world.collide(), "link", "rigid")
    link_world.set_collision_pair_ignored(link, body)
    assert link_world.is_collision_pair_ignored(body, link)
    assert len(link_world.collide()) == 0


def test_simulation_contact_stops_approaching_bodies():
    sx = _simulation()

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


def test_simulation_body_rests_on_static_ground():
    sx = _simulation()

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


def test_simulation_multibody_link_rests_on_static_ground():
    sx = _simulation()

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


def test_simulation_multibody_link_pushes_dynamic_rigid_body():
    sx = _simulation()

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


def test_simulation_multibody_link_contact_friction_stops_slide():
    sx = _simulation()

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


def test_simulation_multibody_link_contact_restitution_bounces():
    sx = _simulation()

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


def test_simulation_contact_restitution():
    sx = _simulation()

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


def test_simulation_contact_friction():
    sx = _simulation()

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


def test_simulation_rigid_body_options_reject_invalid_values():
    sx = _simulation()

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


def test_simulation_loop_closure_rejects_invalid_topology():
    sx = _simulation()

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


def test_simulation_loop_closure_rejects_unsupported_active_policy():
    sx = _simulation()

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


def test_simulation_deformable_body_python_api():
    sx = _simulation()
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


def test_simulation_deformable_body_topology_python_api():
    sx = _simulation()
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


def test_simulation_adaptive_barrier_stiffness_holds_heavy_node_higher():
    sx = _simulation()

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


def test_simulation_world_exposes_deformable_solver_diagnostics():
    sx = _simulation()
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
    # Small direct systems use retained dense LDLT scratch. Sparse Hessian
    # footprint diagnostics only describe sparse/iterative assemblies, so they
    # stay zero here even though the direct solve ran.
    assert after.projected_newton_hessian_nonzeros == 0
    assert after.projected_newton_hessian_storage_bytes == 0
    # This body uses the default direct solve, so the iterative
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


def test_simulation_world_iterative_solver_diagnostic():
    sx = _simulation()
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
    # solve instead of direct factorization.
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
        total_iterative_iterations += diagnostics.projected_newton_iterative_iterations
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


def test_simulation_world_matrix_free_solver_diagnostic():
    sx = _simulation()
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
        total_matrix_free_solves += diagnostics.projected_newton_matrix_free_solves
        total_iterative_iterations += diagnostics.projected_newton_iterative_iterations
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


def test_simulation_world_matrix_free_solver_matches_direct_ground_contact():
    sx = _simulation()

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
            total_iterative_solves += diagnostics.projected_newton_iterative_solves
            total_matrix_free_solves += diagnostics.projected_newton_matrix_free_solves
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

    direct_z, direct_solves, direct_matrix_free, direct_hessian_nonzeros = settle(False)
    matrix_z, matrix_solves, matrix_free_solves, matrix_hessian_nonzeros = settle(True)

    assert direct_solves == 0
    assert direct_matrix_free == 0
    # The one-node direct solve stays under the retained dense cutoff; sparse
    # Hessian footprint remains zero and the solve path is distinguished from
    # matrix-free CG by the iterative-solve counters.
    assert direct_hessian_nonzeros == 0

    assert matrix_solves > 0
    assert matrix_solves == matrix_free_solves
    assert matrix_hessian_nonzeros == 0
    assert 0.0 < matrix_z < 2e-2
    assert matrix_z == pytest.approx(direct_z, abs=1e-9)


def test_simulation_deformable_body_boundary_conditions_python_api():
    sx = _simulation()
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


def test_simulation_world_replay_recording_python_api():
    sx = _simulation()
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


def test_simulation_world_step_profiling_disabled_by_default():
    sx = _simulation()

    world = sx.World()
    assert world.step_profiling_enabled is False

    world.step()

    # Off by default: no per-stage profile is captured.
    assert world.last_step_profile.is_empty()


def _enable_step_profiling_or_skip(world):
    world.step_profiling_enabled = True
    if not world.step_profiling_enabled:
        pytest.skip("DART_BUILD_PROFILE=OFF: World step profiling is compiled out")


def test_simulation_world_step_profiling_records_stages():
    sx = _simulation()

    world = sx.World()
    _enable_step_profiling_or_skip(world)

    world.step()

    profile = world.last_step_profile
    assert isinstance(profile, sx.WorldStepProfile)
    assert profile.is_empty() is False
    assert profile.step_count == 1
    assert profile.wall_time_us >= profile.total_stage_time_us
    assert len(profile.stages) > 0

    stage = profile.get_stage("kinematics")
    assert isinstance(stage, sx.WorldStepStageProfile)
    assert stage.name == "kinematics"
    assert stage.domain == "kinematics"
    assert "task_parallel" in stage.acceleration
    assert isinstance(stage.accelerated_backend_enabled, bool)
    assert stage.duration_us >= 0
    assert stage.duration_ms >= 0.0
    assert stage.total_graph_wall_time_us >= 0
    assert stage.max_graph_worker_count >= 0
    assert stage.max_graph_parallelism >= 0
    assert profile.get_stage("missing") is None

    summary = profile.summary()
    assert "World Step Profile" in summary
    assert "kinematics" in summary
    assert "Execution details" in summary
    assert "(unattributed overhead)" in summary
    assert str(profile) == summary

    if stage.graph_profiles:
        graph_profile = stage.graph_profiles[0]
        assert isinstance(graph_profile, sx.ComputeExecutionProfile)
        assert graph_profile.worker_count >= 1
        assert graph_profile.wall_time_us >= 0
        assert graph_profile.total_node_time_us >= 0
        assert graph_profile.critical_path_time_us >= 0
        assert graph_profile.max_parallelism >= 0
        assert graph_profile.average_parallelism >= 0.0
        assert isinstance(graph_profile.nodes, list)
        if not graph_profile.is_empty():
            assert "Compute Execution Profile" in graph_profile.summary()


def test_simulation_world_step_profiling_accepts_parallel_executor():
    sx = _simulation()

    assert sx.SequentialExecutor().worker_count == 1
    executor = sx.ParallelExecutor(2)
    assert executor.worker_count == 2
    executor.inline_threshold = 0
    assert executor.inline_threshold == 0

    world = sx.World()
    _enable_step_profiling_or_skip(world)

    world.step(executor)

    stage = world.last_step_profile.get_stage("kinematics")
    assert isinstance(stage, sx.WorldStepStageProfile)
    assert stage.graph_profiles
    assert stage.max_graph_worker_count == executor.worker_count
    graph_profile = stage.graph_profiles[0]
    assert isinstance(graph_profile, sx.ComputeExecutionProfile)
    assert graph_profile.worker_count == executor.worker_count
    assert "max_workers=2" in world.last_step_profile.summary()


def test_simulation_world_step_profiling_captures_last_step_for_counts():
    sx = _simulation()

    world = sx.World()
    _enable_step_profiling_or_skip(world)

    world.step(3)

    profile = world.last_step_profile
    assert profile.step_count == 1
    assert profile.wall_time_us >= profile.total_stage_time_us
    assert len(profile.stages) > 0
    assert world.frame == 3


def test_simulation_deformable_scene_loader_python_api(tmp_path):
    sx = _simulation()

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
