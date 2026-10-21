"""DART 7.0 target shape; each feature PR removes its strict xfail marker.

The PR that removes a marker must make its test exercise the full contract in
docs/design/dart7_python_api.md.
M1c owns versioned persistence and older-format fixtures; M1h/M1i own seeded
noise replay; M5 owns detached-capability errors; M6b owns moving frame-attached
cameras. These required feature-PR gates are beyond the M0 smoke scaffolding.

Future tests use the final facade. Builder/solver/sleep/array computation
spellings not yet fixed by that design are provisional M1/M5 signatures.
StepHook uses the provisional simulation submodule (M1g/M5), outside the
curated top-level allowlist.
The unmarked native contact proof is separate and requires no legacy aliases
in the final facade; M5 migrates baseline native tests with the rename table.

The headless-render part needs a graphics device, so it lives in
python/tests/unit/gui/test_target_shape_render.py, which test-gui runs and
test-py leaves out.
"""

import importlib
import logging
import pickle
import sys
import threading
import time

import dartpy as dart
import numpy as np
import pytest


@pytest.fixture
def world():
    return dart.World()


def _make_robot(world):
    # M5 settles joint-spec pose spelling; preserve this independent physical
    # fixture through the final facade, without retaining legacy aliases.
    robot = dart.Multibody(name="robot")
    wrist = robot.add_link("wrist", joint=dart.dynamics.RevoluteJointSpec(
        axis=(1, 0, 0), parent_transform=dart.Transform(translation=(0.1, 0, 0.4))))
    robot.joints[0].name = "shoulder_yaw"
    robot.add_link("palm", parent=wrist, joint=dart.dynamics.RevoluteJointSpec(
        axis=(0, 0, 1), parent_transform=dart.Transform(translation=(0, 0.2, 0.3))))
    world.add(robot)
    return robot


@pytest.fixture
def robot(world):
    return _make_robot(world)


@pytest.mark.xfail(
    strict=True,
    reason="M5/M1f: top-level World, keyword-only construction and the DART detector default are pending",
)
def test_world_construction_keywords():
    world = dart.World(time_step=2e-3, gravity=(0, 0, -4.905))
    assert world.time_step == 2e-3
    np.testing.assert_array_equal(world.gravity, (0, 0, -4.905))
    assert isinstance(world.collision.detector, dart.collision.DARTCollisionDetector)
    explicit = dart.World(name="configured", collision_detector=dart.collision.FCLCollisionDetector(), threads=2)
    assert explicit.name == "configured"
    assert isinstance(explicit.collision.detector, dart.collision.FCLCollisionDetector)
    # M1d/M5 settle executor observation; the explicit count cannot be ignored.
    assert explicit.solver.threads == 2
    with pytest.raises(TypeError):
        dart.World("positional")


@pytest.mark.xfail(
    strict=True,
    reason="M5/M1f: io.load_world and its MJCF DART detector default are pending",
)
def test_load_world_mjcf_detector_default(tmp_path):
    path = tmp_path / "world.xml"
    path.write_text(
        '<mujoco><option timestep="0.002" gravity="0 0 -4.905"/>'
        '<worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>'
    )
    world = dart.io.load_world(path)
    assert isinstance(world.collision.detector, dart.collision.DARTCollisionDetector)
    assert world.time_step == 0.002
    np.testing.assert_array_equal(world.gravity, (0, 0, -4.905))


@pytest.mark.xfail(strict=True, reason="M1e/M5b: WorldConfig typed solver selection is pending")
def test_world_config_solver_selection():
    # M1e/M5b settle the selector and primary_solver observer spelling. Select
    # a primary PGS solver, not Dantzig with a PGS fallback. The separate
    # configuration constructor excludes individual World options.
    config = dart.simulation.WorldConfig(primary_solver=dart.constraint.SolverType.PGS)
    world = dart.World(config=config)
    assert isinstance(world.solver.primary_solver, dart.constraint.PgsBoxedLcpSolver)
    assert isinstance(world.collision.detector, dart.collision.DARTCollisionDetector)
    constructor = dart.World
    with pytest.raises(TypeError):
        constructor(config)
    with pytest.raises(TypeError):
        constructor(config=config, time_step=0.002)


@pytest.mark.xfail(
    strict=True, reason="M1b/M5b: edit/compile modes and frozen topology enforcement are pending"
)
def test_edit_compile(world, robot):
    # M1b/M5b settle builder spelling; the owned-object guard must protect
    # structural mutations as well as World membership changes.
    add_link = robot.add_link
    parent = robot.links["palm"]
    spec = dart.dynamics.RevoluteJointSpec(axis=(0, 0, 1))
    before = (list(robot.links), robot.num_dofs)
    world.compile()
    assert world.is_compiled
    with pytest.raises(Exception) as frozen_link:
        add_link("frozen_child", parent=parent, joint=spec)
    assert type(frozen_link.value).__name__ == "ModelFrozenError"
    assert (list(robot.links), robot.num_dofs) == before
    assert world.is_compiled
    # The design names the error but leaves its public module to M5.
    with pytest.raises(Exception) as frozen:
        world.remove(robot)
    assert type(frozen.value).__name__ == "ModelFrozenError"
    assert robot in world.multibodies
    with world.edit():
        assert not world.is_compiled
        child = add_link("editable_child", parent=parent, joint=spec)
        assert child in robot.links
        assert len(robot.links) == len(before[0]) + 1
        assert robot.num_dofs == before[1] + 1
    assert world.is_compiled
    assert robot in world.multibodies
    world.gravity = [0, 0, 0]
    velocities = np.zeros(robot.num_dofs)
    velocities[-1] = 1
    robot.dofs.velocities = velocities
    positions = robot.dofs.positions.copy()
    world.step()
    assert robot.dofs.positions[-1] > positions[-1]
    assert robot.num_dofs == before[1] + 1
    with world.edit():
        world.remove(robot)
    assert world.is_compiled
    assert robot not in world.multibodies


@pytest.mark.xfail(
    strict=True, reason="M1b/M5b: compile-time rejection of unsupported DART detector shapes is pending"
)
def test_compile_rejects_unsupported_shape(world, robot):
    shape = dart.dynamics.HeightmapShapef()
    shape.set_height_field(np.zeros((2, 2), dtype=np.float32))
    robot.links[0].add_collider(shape)
    world.collision.detector = dart.collision.DARTCollisionDetector()
    compile_world = world.compile
    # No public error type is specified yet; compile must reject, not warn and skip.
    with pytest.raises(Exception):
        compile_world()


@pytest.mark.xfail(
    strict=True, reason="M5: dart.data, io.load_model and World.add are pending"
)
def test_load_model_and_add(world):
    path = dart.data.path("urdf/six_axis_arm/six_axis_arm.urdf")
    robot = world.add(dart.io.load_model(path))
    assert world.multibodies[robot.name] is robot
    assert robot.num_dofs == 6
    assert len(world.multibodies) == 1
    assert world.multibodies[0] == robot
    assert list(world.multibodies) == [robot]
    assert list(world.multibodies.names) == [robot.name]
    with pytest.raises(KeyError) as missing:
        world.multibodies["missing_model"]
    assert robot.name in str(missing.value)
    duplicate = dart.io.load_model(path)
    assert duplicate.name == robot.name
    membership = list(world.multibodies)
    with pytest.raises(Exception) as conflict:
        world.add(duplicate)
    assert type(conflict.value).__name__ == "NameConflictError"
    assert list(world.multibodies) == membership
    assert duplicate not in world.multibodies


@pytest.mark.xfail(strict=True, reason="M5: raising model/world loaders and MJCF world-only loading are pending")
def test_loader_failures(tmp_path):
    load_model = dart.io.load_model
    load_world = dart.io.load_world
    for loader, name in ((load_model, "missing.urdf"), (load_world, "missing.xml")):
        # The exception classes belong to M5; a null return must not succeed.
        with pytest.raises(Exception):
            loader(tmp_path / name)
    path = tmp_path / "world.xml"
    path.write_text('<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>')
    with pytest.raises(Exception):
        load_model(path)


@pytest.mark.xfail(strict=True, reason="M5: load_model fixed_base and explicit package-directory resolution are pending")
def test_load_model_options(tmp_path):
    load_model = dart.io.load_model
    package = tmp_path / "package"
    package.mkdir()
    (package / "mesh.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
        "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n"
    )
    path = tmp_path / "robot.urdf"
    path.write_text(
        '<robot name="packaged"><link name="root"><visual><geometry>'
        '<mesh filename="package://fixture/mesh.obj"/>'
        '</geometry></visual></link></robot>'
    )
    with pytest.raises(Exception):
        load_model(path)
    # M5 settles mapping spelling; explicit package resolution must preserve
    # the actual mesh, and the fixed_base option must change root mobility.
    for fixed, dofs in ((True, 0), (False, 6)):
        model = load_model(path, fixed_base=fixed, package_dirs={"fixture": package})
        assert model.num_dofs == dofs
        assert len(model.links) == 1
        assert len(model.links[0].visuals) == 1
        assert isinstance(model.links[0].visuals[0].shape, dart.dynamics.MeshShape)


@pytest.mark.xfail(strict=True, reason="M5b: World.add/remove and detached ownership for SimpleFrame are pending")
def test_world_frame_membership(world):
    frame = dart.SimpleFrame(name="marker")
    assert world.add(frame) == frame
    assert len(world.frames) == 1
    assert world.frames[0] == frame
    assert world.frames["marker"] == frame
    assert list(world.frames) == [frame]
    assert list(world.frames.names) == ["marker"]
    other = dart.World()
    add = other.add
    with pytest.raises(Exception):
        add(frame)
    assert list(world.frames) == [frame]
    assert len(other.frames) == 0
    world.remove(frame)
    assert len(world.frames) == 0
    assert other.add(frame) == frame
    assert list(other.frames) == [frame]


@pytest.mark.xfail(strict=True, reason="M5: multi-model rejection and named model selection are pending")
def test_load_model_selection(tmp_path, world):
    load_model = dart.io.load_model
    path = tmp_path / "models.sdf"
    path.write_text(
        '<sdf version="1.6"><world name="models">'
        '<model name="first"><link name="first_link"/></model>'
        '<model name="second"><link name="second_root"/>'
        '<link name="second_child"/><joint name="second_joint" type="fixed">'
        '<parent>second_root</parent><child>second_child</child></joint></model>'
        '</world></sdf>'
    )
    # Resolve the pending callable before rejection, so its absence cannot
    # satisfy the required ambiguity error. Exception types belong to M5.
    with pytest.raises(Exception):
        load_model(path)
    for name, links in (("first", ["first_link"]), ("second", ["second_root", "second_child"])):
        model = load_model(path, model=name)
        assert model.name == name
        assert list(model.links.names) == links
        assert world.add(model) is model
        assert world.multibodies[name] == model


@pytest.mark.xfail(strict=True, reason="M5: live add_* builders and immutable keyword-only specifications are pending")
def test_live_model_builders(world):
    # M5 settles collider/visual options and ownership spelling. The live
    # builder and immutable keyword-only joint specification are required.
    spec = dart.dynamics.RevoluteJointSpec(axis=(0, 0, 1))
    with pytest.raises(TypeError):
        dart.dynamics.RevoluteJointSpec((0, 0, 1))
    with pytest.raises(Exception):
        spec.axis = (1, 0, 0)
    model = dart.Multibody(name="built")
    link = model.add_link("root", parent=None, joint=spec)
    shape = dart.dynamics.BoxShape([0.2] * 3)
    collider = link.add_collider(shape=shape)
    visual = link.add_visual(shape=shape)
    assert model.links["root"] == link
    assert model.num_dofs == 1
    for collection in (model.links, model.joints, model.dofs):
        assert len(collection) == 1
        item = collection[0]
        assert list(collection) == [item]
        assert collection[collection.names[0]] == item
        with pytest.raises(KeyError) as missing:
            _ = collection["missing_member"]
        assert collection.names[0] in str(missing.value)
    assert list(link.colliders) == [collider]
    assert list(link.visuals) == [visual]
    assert world.add(model) is model
    other = dart.World()
    with pytest.raises(Exception):
        other.add(model)
    assert model in world.multibodies and model not in other.multibodies
    rigid_link = world.add_rigid_body(name="rigid")
    assert isinstance(rigid_link, dart.Link)
    owners = [owner for owner in world.multibodies if rigid_link in owner.links]
    assert len(owners) == 1
    assert len(owners[0].links) == 1
    world.compile()
    world.step()


@pytest.mark.xfail(strict=True, reason="M5/M6b: final viewer names and removal of the interim gui module are pending")
def test_legacy_gui_removed():
    # Resolve the replacement surface without constructing a graphics device.
    # This keeps a current optional build lacking GUI from falsely passing.
    assert callable(dart.viewer.open)
    assert callable(dart.viewer.launch)
    assert not hasattr(dart, "gui")
    with pytest.raises(ModuleNotFoundError) as missing:
        importlib.import_module("dartpy.gui")
    assert missing.value.name == "dartpy.gui"


@pytest.mark.xfail(strict=True, reason="M5: native Collider/Visual capability handles and collision contexts are pending")
def test_collider_capabilities(world, contact_box):
    link = contact_box.links[0]
    collider, visual = link.colliders[0], link.visuals[0]
    assert isinstance(collider, dart.Collider)
    assert isinstance(visual, dart.Visual)
    assert collider == link.colliders[0]
    assert hash(collider) == hash(link.colliders[0])
    assert visual == link.visuals[0]
    assert hash(visual) == hash(link.visuals[0])
    ground = world.multibodies["ground"].links[0].colliders[0]
    assert collider != ground
    contacts = world.collision.contacts
    assert contacts
    for contact in contacts:
        assert isinstance(contact.collider_a, dart.Collider)
        assert isinstance(contact.collider_b, dart.Collider)
        assert {contact.collider_a, contact.collider_b} == {collider, ground}
    # Distinct nodes on the same Link must retain distinct handle identities.
    link.add_collider(dart.dynamics.BoxShape(size=(0.02,) * 3))
    assert len(link.colliders) == 2
    assert len(set(link.colliders)) == 2
    # The fixture's builder creates both aspects on one node. M5 settles
    # capability setter spelling; both must follow that same live node.
    collider.shape = dart.dynamics.BoxShape(size=(0.25, 0.2, 0.2))
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    translation = np.array([0.01, 0.02, 0.03])
    collider.relative_transform = dart.Transform(rotation=rotation, translation=translation)
    points = np.eye(3)
    for capability in (collider, visual):
        np.testing.assert_array_equal(capability.shape.size, (0.25, 0.2, 0.2))
        np.testing.assert_allclose(capability.relative_transform @ points, points @ rotation.T + translation)


@pytest.mark.xfail(strict=True, reason="M5b: the final World manual-constraint registry is pending")
def test_manual_constraint_registry(world):
    world.gravity = [0, 0, 0]
    body = world.add_rigid_body(name="anchored", joint=dart.dynamics.FreeJointSpec())
    model = world.multibodies["anchored"]
    model.dofs.velocities = [0, 0, 0, 1, 0, 0]
    reference = world.clone()
    anchor = body.world_transform @ np.zeros(3)
    constraint = dart.constraint.BallJointConstraint(link=body, anchor=anchor)
    # Constructor/registry observation spelling is provisional M5b; adding
    # inside edit must retain a constraint that affects the native solve.
    with world.edit():
        world.constraints.add(constraint)
    assert constraint in world.constraints
    constrained = world.clone()
    for _ in range(20):
        world.step()
        reference.step()
        constrained.step()
    np.testing.assert_allclose(body.world_transform @ np.zeros(3), anchor, rtol=0, atol=1e-9)
    cloned_body = constrained.multibodies["anchored"].links[0]
    np.testing.assert_allclose(cloned_body.world_transform @ np.zeros(3), anchor, rtol=0, atol=1e-9)
    free_body = reference.multibodies["anchored"].links[0]
    assert (free_body.world_transform @ np.zeros(3))[0] > anchor[0] + 0.01


@pytest.mark.xfail(strict=True, reason="M5: native diagnostics forwarded to Python's dartpy logger are pending")
def test_native_logging_bridge(caplog):
    sentinel = "dart7 acceptance native warning sentinel"
    # This existing binding invokes C++ common::warn directly; it does not
    # rely on a parser/geometry error whose 7.0 behavior changes.
    with caplog.at_level(logging.WARNING, logger="dartpy"):
        dart.common.warn(sentinel)
    assert any(
        record.name == "dartpy"
        and record.levelno == logging.WARNING
        and sentinel in record.getMessage()
        for record in caplog.records
    )


@pytest.mark.xfail(strict=True, reason="M5/M6b: Material PBR values and Visual material assignment are pending")
def test_material_visual_assignment(robot):
    # M6b settles constructor spelling; these core PBR values and assignment
    # must also work without constructing a renderer or viewer.
    material = dart.Material(
        base_color=(1, 0, 0, 1), metallic=0.25, roughness=0.4, emissive=(0.05, 0, 0)
    )
    np.testing.assert_array_equal(material.base_color, (1, 0, 0, 1))
    assert material.metallic == pytest.approx(0.25)
    assert material.roughness == pytest.approx(0.4)
    np.testing.assert_allclose(material.emissive, (0.05, 0, 0))
    visual = robot.links["wrist"].add_visual(dart.dynamics.BoxShape(size=(0.2,) * 3))
    visual.material = material
    np.testing.assert_array_equal(visual.material.base_color, material.base_color)
    assert visual.material.metallic == material.metallic
    assert visual.material.roughness == material.roughness
    np.testing.assert_array_equal(visual.material.emissive, material.emissive)
    color = material.base_color
    assert not color.flags.writeable
    with pytest.raises(ValueError):
        color[0] = 0
    # M6b settles Material value editing; old exports remain independent.
    material.base_color = (0, 1, 0, 1)
    np.testing.assert_array_equal(color, (1, 0, 0, 1))
    np.testing.assert_array_equal(material.base_color, (0, 1, 0, 1))


@pytest.mark.xfail(
    strict=True, reason="M1g/M5b: joint actuator channels, PositionServo and joint/DOF collections are pending"
)
@pytest.mark.parametrize("target,kd", [(10, 40), (-10, 40), (0.25, 40), (0.05, 0)])
def test_joint_position_servo(world, robot, target, kd):
    world.gravity = [0, 0, 0]
    robot.dofs.positions = np.array([0.2, 0.3])
    robot.dofs.velocities = np.array([0.7, -0.4])
    positions = robot.dofs.positions.copy()
    free_velocities = robot.dofs.velocities.copy()
    h = world.time_step
    inverse_mass = robot.compute_inverse_mass_matrix().copy()
    servo = dart.PositionServo(kp=400, kd=kd, effort_limit=80)
    joint = robot.joints["shoulder_yaw"]
    joint.actuator = servo
    robot.dofs.position_targets = np.array([target, 0.0])
    velocity_target = 0.1
    robot.dofs.velocity_targets = np.array([velocity_target, 0.0])
    # This fixture has constant diagonal inertia and zero Coriolis bias,
    # independently verified natively. The full row uses current q and v.
    compliance = h * kd + h * h * 400
    impulse = np.clip(
        (h * 400 * (target - positions[0]) + h * kd * velocity_target - compliance * free_velocities[0])
        / (1 + compliance * inverse_mass[0, 0]),
        -h * 80,
        h * 80,
    )
    world.step()
    assert robot.dofs.constraint_forces[0] == pytest.approx(impulse / h)
    np.testing.assert_allclose(
        robot.dofs.velocities, free_velocities + inverse_mass[:, 0] * impulse, rtol=0, atol=1e-12
    )


@pytest.mark.xfail(strict=True, reason="M1g/M5b: VelocityServo and persistent velocity targets are pending")
@pytest.mark.parametrize("target,kd", [(100, 40), (-100, 40), (0.05, 40), (0.05, 0)])
def test_joint_velocity_servo(world, robot, target, kd):
    world.gravity = [0, 0, 0]
    robot.dofs.positions = np.array([0.2, 0.3])
    robot.dofs.velocities = np.array([0.7, -0.4])
    free_velocities = robot.dofs.velocities.copy()
    h = world.time_step
    inverse_mass = robot.compute_inverse_mass_matrix().copy()
    # M1g settles constructor and channel spelling. K=0 in the compliant-row
    # equation leaves bounded implicit velocity damping, including zero gain.
    servo = dart.VelocityServo(kd=kd, effort_limit=80)
    robot.joints["shoulder_yaw"].actuator = servo
    robot.dofs.velocity_targets = np.array([target, 0.0])
    impulse = np.clip(
        h * kd * (target - free_velocities[0]) / (1 + h * kd * inverse_mass[0, 0]), -h * 80, h * 80
    )
    world.step()
    assert robot.dofs.constraint_forces[0] == pytest.approx(impulse / h)
    np.testing.assert_allclose(
        robot.dofs.velocities, free_velocities + inverse_mass[:, 0] * impulse, rtol=0, atol=1e-12
    )


@pytest.mark.xfail(strict=True, reason="M1g/M5b: bounded servo rows coupled to hard position limits are pending")
@pytest.mark.parametrize("kind,target,effort", [("position", 10, 80), ("position", 0.11, 4),
                                             ("velocity", 100, 80), ("velocity", 0.1, 4)])
def test_servo_position_limit_priority(world, kind, target, effort):
    world.gravity = [0, 0, 0]
    model = dart.Multibody(name="slider")
    model.add_link("body", joint=dart.dynamics.PrismaticJointSpec(axis=(1, 0, 0)))
    world.add(model)
    joint = model.joints[0]
    # M1g/M5 settle limit setters and the isolated actuator-effort observer.
    # Aggregate constraint forces can cancel motor and limit reactions.
    joint.position_limits = (-1, 0.1)
    joint.limits_enabled = True
    model.dofs.positions = [0.1]
    reference = world.clone()
    free = reference.multibodies["slider"]
    free.joints[0].limits_enabled = False
    for current in (model, free):
        if kind == "position":
            current.joints[0].actuator = dart.PositionServo(kp=400, kd=40, effort_limit=80)
            current.dofs.position_targets = [target]
        else:
            current.joints[0].actuator = dart.VelocityServo(kd=40, effort_limit=80)
            current.dofs.velocity_targets = [target]
    for _ in range(10):
        world.step()
        reference.step()
        np.testing.assert_allclose(model.dofs.positions, [0.1], rtol=0, atol=1e-9)
        np.testing.assert_allclose(model.dofs.velocities, [0], rtol=0, atol=1e-9)
        # At the hard stop v+ is zero: the compliant row reports its full
        # bounded b/h effort, balanced by the limit reaction. The unsaturated
        # cases differ from the free-motion row's inverse-mass denominator.
        assert joint.actuator.effort == pytest.approx(effort, abs=1e-6)
    assert free.dofs.positions[0] > 0.1 + 1e-6


@pytest.mark.xfail(
    strict=True,
    reason="M1h/M5b: native IMUSensor readings and the World sensor registry are pending",
)
def test_imu_sensor(world, robot):
    sensor = dart.sensors.IMUSensor(link=robot.links[0])
    imu = world.sensors.add(sensor)
    assert isinstance(imu, dart.sensors.IMUSensor)
    assert imu is sensor
    assert imu in world.sensors
    robot.dofs.positions = np.full(robot.num_dofs, 0.5)
    robot.dofs.velocities = np.ones(robot.num_dofs)
    world.step()
    assert isinstance(imu.orientation, dart.Rotation)
    half = robot.dofs.positions[0] / 2
    np.testing.assert_allclose(
        imu.orientation.wxyz, (np.cos(half), np.sin(half), 0, 0), rtol=0, atol=1e-12
    )


@pytest.mark.xfail(strict=True, reason="M1h/M5b: native JointStateSensor registration and samples are pending")
def test_joint_state_sensor(world, robot):
    world.gravity = [0, 0, 0]
    joint = robot.joints["shoulder_yaw"]
    robot.dofs.positions = np.array([0.2, 0])
    robot.dofs.velocities = np.array([0.7, 0])
    # M1h settles attachment and reading spelling for the named sensor types.
    sensor = dart.sensors.JointStateSensor(joint=joint)
    assert world.sensors.add(sensor) is sensor
    assert sensor in world.sensors
    world.step()
    assert robot.dofs.positions[0] != 0.2
    assert sensor.position == pytest.approx(robot.dofs.positions[0])
    assert sensor.velocity == pytest.approx(robot.dofs.velocities[0])


@pytest.mark.xfail(strict=True, reason="M1h/M5b: native ForceTorqueSensor registration and samples are pending")
def test_force_torque_sensor(world):
    world.gravity = [0, 0, -9.81]
    body = world.add_rigid_body(name="supported_body", joint=dart.dynamics.WeldJointSpec())
    joint = world.multibodies["supported_body"].joints[0]
    body.mass = 2
    # Attachment and reading spelling are provisional M1h; the support wrench
    # must follow gravity. Magnitude avoids imposing an unstated sign convention.
    sensor = dart.sensors.ForceTorqueSensor(joint=joint)
    assert world.sensors.add(sensor) is sensor
    assert sensor in world.sensors
    for gravity in (9.81, 4.905):
        world.gravity = [0, 0, -gravity]
        world.step()
        assert np.linalg.norm(sensor.force) == pytest.approx(2 * gravity)
        assert np.linalg.norm(sensor.force) == pytest.approx(
            np.linalg.norm(body.body_force[3:])
        )
        np.testing.assert_allclose(sensor.torque, np.zeros(3), rtol=0, atol=1e-12)


@pytest.mark.xfail(strict=True, reason="M1h/M5b: native ContactSensor registration and samples are pending")
def test_contact_sensor(world, contact_box):
    # Attachment and contact payload spelling are provisional M1h.
    sensor = dart.sensors.ContactSensor(link=contact_box.links[0])
    assert world.sensors.add(sensor) is sensor
    assert sensor in world.sensors
    world.step()
    contacts = world.collision.contacts
    expected = sorted(tuple(contact.force) for contact in contacts)
    actual = sorted(tuple(contact.force) for contact in sensor.contacts)
    assert actual and any(np.linalg.norm(force) > 0 for force in actual)
    np.testing.assert_array_equal(actual, expected)
    positions = contact_box.dofs.positions.copy()
    positions[5] = 0.5
    contact_box.dofs.positions = positions
    world.step()
    assert len(world.collision.contacts) == 0
    assert not sensor.contacts


@pytest.mark.xfail(
    strict=True,
    reason="M1d/M5b: step(n), step_count and rejection of bool arguments are pending",
)
def test_step_n(world, robot):
    world.gravity = [0, 0, 0]
    robot.dofs.velocities = np.ones(robot.num_dofs)
    initial = robot.dofs.positions.copy()
    serial = world.clone()
    world.step(20)
    assert world.step_count == 20
    for _ in range(20):
        serial.step()
    assert serial.step_count == world.step_count
    assert serial.time == world.time
    assert not np.array_equal(robot.dofs.positions, initial)
    serial_robot = serial.multibodies["robot"]
    np.testing.assert_array_equal(robot.dofs.positions, serial_robot.dofs.positions)
    np.testing.assert_array_equal(robot.dofs.velocities, serial_robot.dofs.velocities)
    with pytest.raises(TypeError):
        world.step(True)


@pytest.mark.xfail(strict=True, reason="M1d/M5b: GIL-free step(n) is pending")
def test_step_releases_gil(world, robot):
    active = threading.Event()
    finished = threading.Event()
    progress = 0

    def count_during_step():
        nonlocal progress
        active.wait()
        while not finished.is_set():
            progress += 1
            time.sleep(0.001)

    worker = threading.Thread(target=count_during_step)
    worker.start()
    interval = sys.getswitchinterval()
    try:
        # Prevent Python bytecode handoffs before and after the native call.
        sys.setswitchinterval(60)
        active.set()
        try:
            world.step(100_000)
        finally:
            finished.set()
        assert progress > 0
    finally:
        finished.set()
        active.set()
        sys.setswitchinterval(interval)
        worker.join(timeout=5)
        assert not worker.is_alive()


@pytest.mark.xfail(
    strict=True, reason="M1d/M1g/M5b: StepHook and WorldBusyError access guards are pending"
)
def test_world_busy_during_step(world, robot):
    active = threading.Event()
    read_finished = threading.Event()
    errors = []

    def read_during_step():
        if not active.wait(timeout=5):
            return
        try:
            _ = world.time
        except Exception as error:
            errors.append(type(error).__name__)
        else:
            errors.append("unguarded read")
        finally:
            read_finished.set()

    def during_step(current):
        # Event.wait releases the GIL while this callback keeps the World in
        # its stepping boundary, excluding valid idle reads before and after.
        active.set()
        assert read_finished.wait(timeout=5)

    hook = dart.simulation.StepHook(during_step, every=1)
    world.hooks.add(hook)
    worker = threading.Thread(target=read_during_step)
    worker.start()
    try:
        world.step()
        assert errors == ["WorldBusyError"]
    finally:
        active.set()
        worker.join(timeout=5)
        assert not worker.is_alive()


@pytest.mark.xfail(strict=True, reason="M1d/M1g/M5b: StepHook and re-entrant stepping guards are pending")
def test_world_rejects_reentrant_step(world, robot):
    entries = 0

    def during_step(current):
        nonlocal entries
        entries += 1
        # Bound a broken implementation to one nested call, rather than
        # letting the acceptance check recurse until the process fails.
        if entries > 1:
            return
        count, time = current.step_count, current.time
        positions = robot.dofs.positions.copy()
        velocities = robot.dofs.velocities.copy()
        # The design requires rejection but leaves its exception type to M1d.
        with pytest.raises(Exception):
            current.step()
        assert (current.step_count, current.time) == (count, time)
        np.testing.assert_array_equal(robot.dofs.positions, positions)
        np.testing.assert_array_equal(robot.dofs.velocities, velocities)

    # Constructor/callback spelling remains provisional M1g.
    world.hooks.add(dart.simulation.StepHook(during_step, every=1))
    world.step()
    assert entries == 1
    assert world.step_count == 1


@pytest.mark.xfail(strict=True, reason="M5b: final World lifecycle properties are pending")
def test_world_reset(world, robot, contact_box):
    robot.dofs.velocities = np.ones(robot.num_dofs)
    initial = robot.dofs.positions.copy()
    for _ in range(3):
        world.step()
    assert not np.array_equal(robot.dofs.positions, initial)
    assert world.time > 0 and world.step_count > 0
    assert len(world.collision.contacts) > 0
    assert np.linalg.norm(contact_box.dofs.constraint_forces) > 0
    members = list(world.multibodies)
    states = [(model.dofs.positions.copy(), model.dofs.velocities.copy()) for model in (robot, contact_box)]
    # Existing reset semantics clear clocks/history/impulses, preserving q/v.
    # The final design lists reset without redefining it as an initial-state reset.
    world.reset()
    assert world.time == 0 and world.step_count == 0
    assert list(world.multibodies) == members
    assert len(world.collision.contacts) == 0
    for model, (positions, velocities) in zip((robot, contact_box), states):
        np.testing.assert_array_equal(model.dofs.positions, positions)
        np.testing.assert_array_equal(model.dofs.velocities, velocities)
        np.testing.assert_array_equal(model.dofs.constraint_forces, np.zeros(model.num_dofs))
    world.step()
    assert world.step_count == 1
    assert world.time == pytest.approx(world.time_step)
    assert not np.array_equal(robot.dofs.positions, states[0][0])


@pytest.mark.xfail(strict=True, reason="M1g/M5b: StepHook and the World hook registry are pending")
def test_step_hook_every(world, robot):
    calls = []

    # M1g settles the callback signature; decimation and global boundaries are
    # the acceptance contract regardless of the final registration spelling.
    hook = dart.simulation.StepHook(lambda world: calls.append(world.step_count), every=3)
    world.hooks.add(hook)
    world.step(4)
    first = calls.copy()
    world.step(5)
    assert first and len(calls) == 3
    assert np.diff(calls).tolist() == [3, 3]
    assert calls[0] in (0, 3)
    assert all(count % 3 == 0 for count in calls)


@pytest.mark.xfail(strict=True, reason="M5b: DOF state and target properties with read-only copies are pending")
@pytest.mark.parametrize("channel", ["positions", "velocities", "position_targets", "velocity_targets"])
def test_dof_arrays_are_read_only_copies(robot, channel):
    expected = np.full(robot.num_dofs, 0.5)
    setattr(robot.dofs, channel, expected)
    array = getattr(robot.dofs, channel)
    assert not array.flags.writeable
    with pytest.raises(ValueError):
        array[0] = -0.5
    np.testing.assert_array_equal(getattr(robot.dofs, channel), expected)
    # Changing native storage must not mutate an already returned copy.
    setattr(robot.dofs, channel, -expected)
    np.testing.assert_array_equal(array, expected)


@pytest.mark.xfail(strict=True, reason="M5: non-DOF array properties with read-only copies are pending")
def test_gravity_is_read_only_copy(world):
    expected = np.array([0.5, -0.25, -4.905])
    world.gravity = expected
    gravity = world.gravity
    np.testing.assert_array_equal(gravity, expected)
    assert not gravity.flags.writeable
    with pytest.raises(ValueError):
        gravity[0] = 0
    world.gravity = -expected
    np.testing.assert_array_equal(gravity, expected)
    np.testing.assert_array_equal(world.gravity, -expected)


@pytest.mark.xfail(strict=True, reason="M5: immutable Transform composition is pending")
def test_transform_composition():
    transform = dart.Transform
    # A quarter turn about x keeps the composition order observable.
    half = np.pi / 4
    parent = dart.Transform(
        wxyz=(np.cos(half), np.sin(half), 0, 0), translation=(1, 2, 3)
    )
    composed = parent @ dart.Transform(translation=(0, 0, 0.1))
    np.testing.assert_allclose(composed @ np.zeros(3), (1, 1.9, 3))
    quaternion = parent.wxyz.copy()
    with pytest.raises(ValueError):
        parent.wxyz[0] = 0
    np.testing.assert_array_equal(parent.wxyz, quaternion)
    np.testing.assert_array_equal(parent @ np.zeros(3), (1, 2, 3))
    # M5 calibrates tolerances and settles constructor forms. These gross
    # invalid inputs must raise ValueError rather than enter the simulation.
    reflection = np.diag([-1, 1, 1])
    shear = np.array([[1, 0.25, 0], [0, 1, 0], [0, 0, 1]])
    for options in (
        {"translation": (np.inf, 0, 0)},
        {"wxyz": (np.nan, 0, 0, 0)},
        {"rotation": np.diag([2, 1, 1])},
        {"rotation": reflection},
        {"rotation": np.eye(3), "wxyz": (1, 0, 0, 0)},
        {"rotation": reflection, "normalize": True},
        {"rotation": shear, "normalize": True},
    ):
        with pytest.raises(ValueError):
            transform(**options)
    projected = transform(rotation=1.001 * np.eye(3), normalize=True)
    np.testing.assert_allclose(projected @ np.eye(3), np.eye(3), rtol=0, atol=1e-12)


@pytest.mark.xfail(strict=True, reason="M5: immutable Rotation values and point application are pending")
def test_rotation_value():
    half = np.pi / 4
    # Construction spelling is provisional M5; the rotation convention and
    # immutable quaternion value are required independently of sensor output.
    rotation = dart.Rotation(wxyz=(np.cos(half), np.sin(half), 0, 0))
    np.testing.assert_allclose(rotation @ np.array([0, 1, 0]), (0, 0, 1), rtol=0, atol=1e-12)
    before = rotation.wxyz.copy()
    with pytest.raises(ValueError):
        rotation.wxyz[0] = 0
    with pytest.raises(Exception):
        rotation.wxyz = (1, 0, 0, 0)
    np.testing.assert_array_equal(rotation.wxyz, before)
    asymmetric = np.array([1, 2, 3, 4], dtype=float) / np.sqrt(30)
    reordered = asymmetric[[1, 2, 3, 0]]
    for value_type in (dart.Rotation, dart.Transform):
        scalar_first = value_type(wxyz=asymmetric)
        scalar_last = value_type(xyzw=reordered)
        np.testing.assert_allclose(scalar_first @ np.eye(3), scalar_last @ np.eye(3), rtol=0, atol=1e-15)
        np.testing.assert_allclose(scalar_last.wxyz, asymmetric, rtol=0, atol=1e-15)
        np.testing.assert_allclose(scalar_first.xyzw, reordered, rtol=0, atol=1e-15)


@pytest.mark.xfail(
    strict=True,
    reason="M5: links, Link.world_transform and immutable Transform are pending",
)
def test_link_world_transform(robot):
    # The x-axis shoulder keeps the rotation from commuting with the z offset,
    # so a reversed composition order also fails.
    robot.dofs.positions = np.full(robot.num_dofs, 0.5)
    X_WP = robot.links["palm"].world_transform @ dart.Transform(translation=(0, 0, 0.1))
    cosine, sine = np.cos(0.5), np.sin(0.5)
    shoulder = np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])
    wrist = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
    rotation = shoulder @ wrist
    translation = np.array([0.1, 0, 0.4]) + shoulder @ np.array([0, 0.2, 0.3])
    points = np.eye(3)
    expected = (points + (0, 0, 0.1)) @ rotation.T + translation
    np.testing.assert_allclose(X_WP @ points, expected, rtol=0, atol=1e-12)


@pytest.mark.xfail(
    strict=True,
    reason="M1c/M1g/M5b: dynamic snapshots including persistent servo targets are pending",
)
def test_capture_restore_state(world, robot):
    robot.joints["shoulder_yaw"].actuator = dart.PositionServo(
        kp=400, kd=40, effort_limit=80
    )
    targets = np.full(robot.num_dofs, 0.5)
    velocity_targets = np.full(robot.num_dofs, 0.25)
    robot.dofs.positions = [0.2, -0.3]
    robot.dofs.position_targets = targets
    robot.dofs.velocity_targets = velocity_targets
    robot.dofs.velocities = np.ones(robot.num_dofs)
    positions = robot.dofs.positions.copy()
    velocities = robot.dofs.velocities.copy()
    time = world.time
    state = world.capture_state()
    clone = world.clone()
    clone_robot = clone.multibodies["robot"]
    clone.gravity = [0, 0, 0]
    clone_robot.dofs.positions = -positions + 0.2
    clone_robot.dofs.velocities = -velocities
    clone_robot.dofs.position_targets = -targets
    clone_robot.dofs.velocity_targets = -velocity_targets
    clone.restore_state(state)
    np.testing.assert_array_equal(clone.gravity, (0, 0, 0))
    np.testing.assert_array_equal(clone_robot.dofs.positions, positions)
    np.testing.assert_array_equal(clone_robot.dofs.velocities, velocities)
    np.testing.assert_array_equal(clone_robot.dofs.position_targets, targets)
    np.testing.assert_array_equal(clone_robot.dofs.velocity_targets, velocity_targets)
    assert clone.time == time
    # Matching digests cannot depend on clone lineage or native object identity.
    rebuilt = dart.World()
    rebuilt_robot = _make_robot(rebuilt)
    rebuilt_robot.joints["shoulder_yaw"].actuator = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    rebuilt.step(3)
    assert rebuilt.time != time
    rebuilt_robot.dofs.positions = -positions
    rebuilt_robot.dofs.velocities = -velocities
    rebuilt_robot.dofs.position_targets = -targets
    rebuilt_robot.dofs.velocity_targets = -velocity_targets
    rebuilt.restore_state(state)
    for channel, expected in (
        ("positions", positions), ("velocities", velocities),
        ("position_targets", targets), ("velocity_targets", velocity_targets),
    ):
        np.testing.assert_array_equal(getattr(rebuilt_robot.dofs, channel), expected)
    assert rebuilt.time == time
    # Equal array sizes do not imply a matching structural digest.
    different = world.clone()
    different_robot = different.multibodies["robot"]
    with different.edit():
        different.add_rigid_body(name="extra_fixed_model", joint=dart.dynamics.WeldJointSpec())
        fixed = different.multibodies["extra_fixed_model"]
    assert fixed.num_dofs == 0
    different_robot.dofs.positions = np.full(robot.num_dofs, 0.25)
    different_robot.dofs.velocities = -velocities
    different_robot.dofs.position_targets = -targets
    different_robot.dofs.velocity_targets = -velocity_targets
    before = (
        different_robot.dofs.positions.tobytes(),
        different_robot.dofs.velocities.tobytes(),
        different_robot.dofs.position_targets.tobytes(),
        different_robot.dofs.velocity_targets.tobytes(),
        different.time,
    )
    with pytest.raises(Exception):
        different.restore_state(state)
    assert (
        different_robot.dofs.positions.tobytes(),
        different_robot.dofs.velocities.tobytes(),
        different_robot.dofs.position_targets.tobytes(),
        different_robot.dofs.velocity_targets.tobytes(),
        different.time,
    ) == before
    robot.dofs.position_targets = np.full(robot.num_dofs, -0.5)
    robot.dofs.velocity_targets = -velocity_targets
    np.testing.assert_array_equal(robot.dofs.position_targets, -targets)
    world.step()
    assert not np.array_equal(robot.dofs.positions, positions)
    world.restore_state(state)
    np.testing.assert_array_equal(robot.dofs.positions, positions)
    np.testing.assert_array_equal(robot.dofs.velocities, velocities)
    np.testing.assert_array_equal(robot.dofs.position_targets, targets)
    np.testing.assert_array_equal(robot.dofs.velocity_targets, velocity_targets)
    assert world.time == time


@pytest.mark.xfail(
    strict=True, reason="M1c/M5b: exact checkpoints, bit-identical replay and mismatch rejection are pending"
)
@pytest.mark.parametrize("solver,other_solver", [("DANTZIG", "PGS"), ("PGS", "DANTZIG")])
def test_capture_restore_checkpoint(world, contact_box, solver, other_solver):
    robot = contact_box

    # M1e/M1c/M5 settle typed solver mutation and sleep settings on subobjects.
    def use_solver(target, name):
        target.solver.primary_solver = getattr(dart.constraint.SolverType, name)
        solver_class = {"DANTZIG": "DantzigBoxedLcpSolver", "PGS": "PgsBoxedLcpSolver"}[name]
        assert isinstance(target.solver.primary_solver, getattr(dart.constraint, solver_class))

    def configure(target):
        use_solver(target, solver)
        target.solver.sleep.enabled = True
        target.solver.sleep.time_until_sleep = 5 * target.time_step

    configure(world)
    for _ in range(2):
        world.step()
    assert not robot.is_resting
    checkpoint = world.capture_checkpoint()

    def frame():
        contacts = world.collision.contacts
        contact_state = tuple(
            (
                contact.point.tobytes(),
                contact.normal.tobytes(),
                contact.force.tobytes(),
                contact.penetration_depth,
            )
            for contact in contacts
        )
        # Serialized fields plus solver outputs and sleep transitions are the oracle.
        return (
            pickle.dumps(world.capture_checkpoint(), protocol=5),
            world.time,
            world.step_count,
            robot.dofs.positions.tobytes(),
            robot.dofs.velocities.tobytes(),
            robot.dofs.accelerations.tobytes(),
            robot.dofs.forces.tobytes(),
            (robot.dofs.constraint_forces * world.time_step).tobytes(),
            contact_state,
            robot.is_resting,
        )

    initial = frame()
    expected = []
    for _ in range(20):
        world.step()
        expected.append(frame())
    assert robot.is_resting
    assert expected[-1] != initial
    world.restore_state(pickle.loads(pickle.dumps(checkpoint, protocol=5)))
    assert frame() == initial
    for expected_frame in expected:
        world.step()
        assert frame() == expected_frame

    # Exact checkpoints restore only into the World that captured them, so an
    # otherwise identical clone, or a World rebuilt with the same history and
    # settings, rejects one. The design specifies no error type for a
    # different World. Resolve APIs before raises.
    rebuilt = dart.World()
    _make_contact_box(rebuilt)
    configure(rebuilt)
    for other in (world.clone(), rebuilt):
        restore = other.restore_state
        before = pickle.dumps(other.capture_checkpoint(), protocol=5)
        with pytest.raises(Exception):
            restore(checkpoint)
        assert pickle.dumps(other.capture_checkpoint(), protocol=5) == before

    # Each mismatch is rejected on the capturing World, against a checkpoint
    # taken one step before that one change: no earlier change can satisfy it,
    # and a restore that writes state before raising changes the frame. The
    # added fixed model keeps every state-array size equal.
    for change in ("topology", "solver", "parameters"):
        captured = world.capture_checkpoint()
        world.step()
        if change == "topology":
            with world.edit():
                world.add_rigid_body(name="checkpoint_extra_fixed", joint=dart.dynamics.WeldJointSpec())
        elif change == "solver":
            # Replacing the solver is a topology edit.
            with world.edit():
                use_solver(world, other_solver)
        else:
            world.gravity = [0, 0, 0]
        before_rejection = frame()
        with pytest.raises(Exception) as mismatch:
            world.restore_state(captured)
        assert type(mismatch.value).__name__ == "CheckpointMismatchError"
        assert frame() == before_rejection

    # A World that differs only in its detector, such as FCL, cannot capture
    # one.
    fcl = dart.World(collision_detector=dart.collision.FCLCollisionDetector())
    _make_contact_box(fcl)
    configure(fcl)
    for _ in range(2):
        fcl.step()
    capture = fcl.capture_checkpoint
    with pytest.raises(Exception):
        capture()


class _CheckpointControl:
    def __init__(self):
        self.calls = 0

    def __call__(self, current):
        self.calls += 1
        current.multibodies["robot"].dofs.velocities = [0.01 * self.calls, -0.005 * self.calls]

    def __getstate__(self):
        return {"calls": self.calls}

    def __setstate__(self, state):
        self.calls = state["calls"]


class _UnsupportedCheckpointControl(_CheckpointControl):
    def __getstate__(self):
        raise TypeError("hook state cannot be serialized")


@pytest.mark.xfail(strict=True, reason="M1c/M1g: serializable hook checkpoint state and unsupported-state rejection are pending")
def test_checkpoint_hook_state(world, robot):
    world.gravity = [0, 0, 0]
    control = _CheckpointControl()
    # M1g settles the wrapper/callback spelling; the callable's declared
    # Python state must affect exact replay rather than being silently dropped.
    world.hooks.add(dart.simulation.StepHook(control, every=1))
    for _ in range(3):
        world.step()
    assert control.calls == 3
    checkpoint = world.capture_checkpoint()

    def frame():
        return (
            robot.dofs.positions.tobytes(),
            robot.dofs.velocities.tobytes(),
            world.time,
            pickle.dumps(world.capture_checkpoint(), protocol=5),
        )

    initial = frame()
    expected = []
    for _ in range(6):
        world.step()
        expected.append(frame())
    assert expected[-1] != initial
    world.restore_state(pickle.loads(pickle.dumps(checkpoint, protocol=5)))
    assert frame() == initial
    for expected_frame in expected:
        world.step()
        assert frame() == expected_frame
    # Stepping and checkpoint replay prepare a lazy World without freezing it.
    assert not world.is_compiled
    world.hooks.add(dart.simulation.StepHook(_UnsupportedCheckpointControl(), every=1))
    capture = world.capture_checkpoint
    with pytest.raises(Exception):
        capture()


@pytest.fixture
def contact_box(world):
    return _make_contact_box(world)


def _make_contact_box(world):
    # Sleep configuration and shared-node visual construction spelling are
    # provisional M1c/M5. Native setup proofs below remain separate.
    world.solver.sleep.enabled = False
    ground = world.add_rigid_body(name="ground", joint=dart.dynamics.WeldJointSpec(
        parent_transform=dart.Transform(translation=(0, 0, -0.05))))
    ground.add_collider(dart.dynamics.BoxShape(size=(2, 2, 0.1)), visual=True)
    body = world.add_rigid_body(name="box", joint=dart.dynamics.FreeJointSpec())
    body.add_collider(dart.dynamics.BoxShape(size=(0.2, 0.2, 0.2)), visual=True)
    box = world.multibodies["box"]
    box.dofs.positions = np.array([0, 0, 0, 0, 0, 0.3])
    for _ in range(2000):
        world.step()
    # Only explicit compile() freezes topology; ordinary steps stay lazy.
    assert not world.is_compiled
    return box





@pytest.mark.xfail(
    strict=True, reason="M1i/M1g/M5b: WorldBatch clones, set/step and the position-target channel are pending"
)
def test_world_batch_set_step(world, robot):
    world.gravity = [0, 0, 0]
    robot.dofs.velocities = np.zeros(robot.num_dofs)
    servo = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    robot.joints["shoulder_yaw"].actuator = servo
    batch = dart.WorldBatch(world, count=64)
    assert batch.positions.shape == (64, robot.num_dofs)
    batch.set("velocities", np.zeros_like(batch.positions))
    targets = np.zeros_like(batch.positions)
    targets[:, 0] = np.linspace(-0.5, 0.5, 64)
    batch.set("position_targets", targets)
    batch.step(20)
    expected = []
    for target in targets:
        serial = world.clone()
        serial_robot = serial.multibodies["robot"]
        serial_robot.dofs.position_targets = target
        for _ in range(20):
            serial.step()
        expected.append(serial_robot.dofs.positions.copy())
    assert expected[0][0] < 0 < expected[-1][0]
    np.testing.assert_array_equal(batch.positions, expected)
    # The source is a template; stepping its independent clones leaves it alone.
    np.testing.assert_array_equal(robot.dofs.positions, np.zeros(robot.num_dofs))


@pytest.mark.xfail(strict=True, reason="M1c/M1i: selected-row WorldBatch snapshot broadcasting is pending")
def test_world_batch_restore_state(world, robot):
    robot.dofs.positions = [0.2, -0.3]
    robot.dofs.velocities = [0.4, -0.5]
    robot.dofs.position_targets = [0.6, -0.7]
    robot.dofs.velocity_targets = [0.8, -0.9]
    for _ in range(3):
        world.step()
    channels = ("positions", "velocities", "position_targets", "velocity_targets")
    np.testing.assert_array_equal(robot.dofs.position_targets, [0.6, -0.7])
    np.testing.assert_array_equal(robot.dofs.velocity_targets, [0.8, -0.9])
    source = {name: getattr(robot.dofs, name).copy() for name in channels}
    source["time"] = world.time
    assert source["time"] > 0
    state = world.capture_state()
    batch = dart.WorldBatch(world, count=4)
    rows = np.array([[1, 2], [3, 4], [5, 6], [7, 8]], dtype=float)
    requested = {"positions": rows, "velocities": -rows,
                 "position_targets": 2 * rows, "velocity_targets": -2 * rows,
                 "time": np.array([1, 2, 3, 4], dtype=float)}
    # M1i settles the scalar-clock field and array observer spelling.
    observers = {name: name for name in channels} | {"time": "times"}
    before = {}
    for name, values in requested.items():
        batch.set(name, values)
        before[name] = getattr(batch, observers[name]).copy()
        np.testing.assert_array_equal(before[name], values)
    batch.restore_state(state, indices=[0, 2])
    for name, values in before.items():
        expected = values.copy()
        expected[[0, 2]] = source[name]
        np.testing.assert_array_equal(getattr(batch, observers[name]), expected)
    for name in channels:
        np.testing.assert_array_equal(getattr(robot.dofs, name), source[name])
    assert world.time == source["time"]


@pytest.mark.xfail(strict=True, reason="M1i: scalar and selected WorldBatch writes are pending")
def test_world_batch_selected_writes(world, robot):
    batch = dart.WorldBatch(world, count=4)
    expected = np.full((4, robot.num_dofs), 0.5)
    batch.set("positions", 0.5)
    np.testing.assert_array_equal(batch.positions, expected)
    selected = np.array([[-0.2, 0.3], [0.4, -0.5]])
    batch.set("positions", selected, indices=[0, 2])
    expected[[0, 2]] = selected
    np.testing.assert_array_equal(batch.positions, expected)
    mask = np.array([False, True, False, True])
    batch.set("positions", -0.75, mask=mask)
    expected[mask] = -0.75
    np.testing.assert_array_equal(batch.positions, expected)
    np.testing.assert_array_equal(robot.dofs.positions, np.zeros(robot.num_dofs))


@pytest.mark.xfail(strict=True, reason="M1i: per-clone batch parameters without recompilation are pending")
def test_world_batch_parameters(world, robot):
    world.gravity = [0, 0, 0]
    robot.dofs.velocities = [0.5, -0.25]
    positions = robot.dofs.positions.copy()
    velocities = robot.dofs.velocities.copy()
    time_step = world.time_step
    batch = dart.WorldBatch(world, count=4)
    # M1i settles argument spelling; distinct per-row time steps must affect
    # execution, rather than silently retaining the template's configuration.
    batch.set_parameter("time_step", np.array([2, 4]) * time_step, indices=[0, 2])
    batch.step(20)
    expected = []
    for multiplier in (2, 1, 4, 1):
        serial = world.clone()
        serial.time_step = multiplier * time_step
        for _ in range(20):
            serial.step()
        expected.append(serial.multibodies["robot"].dofs.positions.copy())
    assert not np.array_equal(expected[0], expected[1])
    assert not np.array_equal(expected[0], expected[2])
    np.testing.assert_array_equal(batch.positions, expected)
    assert world.time_step == time_step
    assert world.time == 0
    np.testing.assert_array_equal(robot.dofs.positions, positions)
    np.testing.assert_array_equal(robot.dofs.velocities, velocities)


@pytest.mark.xfail(strict=True, reason="M1i/M1g/M5b: remapping clones with World-bound step hooks are pending")
def test_world_batch_remaps_hook(world, robot):
    world.gravity = [0, 0, 0]
    robot.joints["shoulder_yaw"].actuator = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    robot.dofs.position_targets = np.zeros(robot.num_dofs)

    def drive(current):
        # Stateless and bound through the callback's World, not a source model
        # captured in a closure. M1g settles the callback signature.
        current.multibodies["robot"].dofs.position_targets = np.array([0.5, 0])

    world.hooks.add(dart.simulation.StepHook(drive, every=1))
    before_time = world.time
    batch = dart.WorldBatch(world, count=3)
    batch.step(20)
    assert world.time == before_time
    np.testing.assert_array_equal(robot.dofs.positions, np.zeros(robot.num_dofs))
    np.testing.assert_array_equal(robot.dofs.position_targets, np.zeros(robot.num_dofs))
    # Step the actual source as oracle: a shallow clone that drops the hook
    # cannot make both the batch and a cloned serial oracle fail identically.
    for _ in range(20):
        world.step()
    assert robot.dofs.positions[0] > 0
    np.testing.assert_array_equal(batch.positions, np.tile(robot.dofs.positions, (3, 1)))


# Current native fixture evidence is separate from the final public facade.
def _native_contact_box(world):
    world.getConstraintSolver().setCollisionDetector(
        dart.collision.DARTCollisionDetector()
    )
    options = world.getDeactivationOptions()
    options.mEnabled = False
    world.setDeactivationOptions(options)
    ground = dart.dynamics.Skeleton("ground")
    joint, body = ground.createWeldJointAndBodyNodePair()
    joint.setTransformFromParentBodyNode(dart.math.Isometry3(np.eye(3), [0, 0, -0.05]))
    node = body.createShapeNode(dart.dynamics.BoxShape([2, 2, 0.1]))
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    ground.setMobile(False)
    world.addSkeleton(ground)
    box = dart.dynamics.Skeleton("box")
    joint, body = box.createFreeJointAndBodyNodePair()
    joint.setPositions([0, 0, 0, 0, 0, 0.3])
    node = body.createShapeNode(dart.dynamics.BoxShape([0.2, 0.2, 0.2]))
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    world.addSkeleton(box)
    for _ in range(2000):
        world.step()
    return box

@pytest.mark.skipif(
    not hasattr(dart.dynamics, "Skeleton"),
    reason="Baseline native scene proof; M5 retires or migrates it after removing the legacy binding surface",
)
@pytest.mark.parametrize("solver", ["DANTZIG", "PGS"])
def test_checkpoint_contact_scene(solver):
    world = dart.simulation.World()
    constraint_solver = world.getConstraintSolver()
    if solver == "PGS":
        # The legacy binding cannot construct a PGS solver, but the default
        # secondary solver is one.
        pgs = constraint_solver.getSecondaryBoxedLcpSolver()
        constraint_solver.setSecondaryBoxedLcpSolver(constraint_solver.getBoxedLcpSolver())
        constraint_solver.setBoxedLcpSolver(pgs)
    contact_box = _native_contact_box(world)
    expected = {"DANTZIG": "DantzigBoxedLcpSolver", "PGS": "PgsBoxedLcpSolver"}[solver]
    assert type(constraint_solver.getBoxedLcpSolver()).__name__ == expected
    # This setup gate runs without xfail, so a broken scene cannot hide behind M1c.
    result = world.getLastCollisionResult()
    assert result.getNumContacts() > 0
    assert any(np.linalg.norm(result.getContact(i).force) > 0 for i in range(result.getNumContacts()))
    assert np.linalg.norm(contact_box.getConstraintForces()) > 0
    assert not contact_box.isResting()
    options = world.getDeactivationOptions()
    options.mEnabled = True
    options.mTimeUntilSleep = 5 * world.getTimeStep()
    world.setDeactivationOptions(options)
    for _ in range(2):
        world.step()
    assert not contact_box.isResting()
    for _ in range(20):
        world.step()
    assert contact_box.isResting()
