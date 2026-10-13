"""DART 7.0 target shape; each feature PR removes its strict xfail marker.

The PR that removes a marker must make its test exercise the full contract in
docs/design/dart7_python_api.md.

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
    # Existing constructors keep each failure local to the feature under test.
    return dart.simulation.World()


@pytest.fixture
def robot(world):
    # Joint offsets put the wrist off the origin and the palm off the wrist, so
    # a link's world pose differs from its pose in the parent frame and from
    # its parent's pose.
    robot = dart.dynamics.Skeleton("robot")
    joint, wrist = robot.createRevoluteJointAndBodyNodePair()
    joint.setName("shoulder_yaw")
    joint.setAxis([1, 0, 0])
    joint.setTransformFromParentBodyNode(dart.math.Isometry3(np.eye(3), [0.1, 0, 0.4]))
    wrist.setName("wrist")
    joint, palm = robot.createRevoluteJointAndBodyNodePair(wrist)
    joint.setTransformFromParentBodyNode(dart.math.Isometry3(np.eye(3), [0, 0.2, 0.3]))
    palm.setName("palm")
    world.addSkeleton(robot)
    return robot


@pytest.mark.xfail(
    strict=True,
    reason="M5/M1f: top-level World, keyword-only construction and the DART detector default are pending",
)
def test_world_construction_keywords():
    world = dart.World(time_step=1e-3, gravity=(0, 0, -9.81))
    assert world.time_step == 1e-3
    np.testing.assert_array_equal(world.gravity, (0, 0, -9.81))
    assert isinstance(world.collision.detector, dart.collision.DARTCollisionDetector)
    with pytest.raises(TypeError):
        dart.World("positional")


@pytest.mark.xfail(
    strict=True,
    reason="M5/M1f: io.load_world and its MJCF DART detector default are pending",
)
def test_load_world_mjcf_detector_default(tmp_path):
    path = tmp_path / "world.xml"
    path.write_text(
        '<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>'
    )
    world = dart.io.load_world(path)
    assert isinstance(world.collision.detector, dart.collision.DARTCollisionDetector)


@pytest.mark.xfail(strict=True, reason="M1e/M5b: WorldConfig typed solver selection is pending")
def test_world_config_solver_selection():
    # M1e/M5b settle the selector, construction and primary_solver observer
    # spelling. Select a primary PGS solver, not Dantzig with a PGS fallback.
    config = dart.simulation.WorldConfig(primary_solver=dart.constraint.SolverType.PGS)
    world = dart.World(config=config)
    assert isinstance(world.solver.primary_solver, dart.constraint.PgsBoxedLcpSolver)


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
    world.setGravity([0, 0, 0])
    velocities = np.zeros(robot.num_dofs)
    velocities[-1] = 1
    robot.setVelocities(velocities)
    positions = robot.getPositions().copy()
    world.step()
    assert robot.getPositions()[-1] > positions[-1]
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
    shape.setHeightField(np.zeros((2, 2), dtype=np.float32))
    node = robot.getRootBodyNode().createShapeNode(shape)
    node.createCollisionAspect()
    world.getConstraintSolver().setCollisionDetector(
        dart.collision.DARTCollisionDetector()
    )
    compile_world = world.compile
    # No public error type is specified yet; compile must reject, not warn and skip.
    with pytest.raises(Exception):
        compile_world()


@pytest.mark.xfail(
    strict=True, reason="M5: dart.data, io.load_model and World.add are pending"
)
def test_load_model_and_add(world):
    path = dart.data.path("urdf/KR5/KR5 sixx R650.urdf")
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
    model = dart.dynamics.Multibody(name="built")
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
            collection["missing_member"]
        assert collection.names[0] in str(missing.value)
    assert list(link.colliders) == [collider]
    assert list(link.visuals) == [visual]
    assert world.add(model) is model
    other = dart.simulation.World()
    with pytest.raises(Exception):
        other.add(model)
    assert model in world.multibodies and model not in other.multibodies
    rigid_link = world.add_rigid_body(name="rigid")
    assert isinstance(rigid_link, dart.dynamics.Link)
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
    contacts = world.getLastCollisionResult()
    assert contacts.getNumContacts() > 0
    for i in range(contacts.getNumContacts()):
        contact = contacts.getContact(i)
        assert isinstance(contact.collider_a, dart.Collider)
        assert isinstance(contact.collider_b, dart.Collider)
        assert {contact.collider_a, contact.collider_b} == {collider, ground}
    # Distinct nodes on the same Link must retain distinct handle identities.
    other = contact_box.getRootBodyNode().createShapeNode(dart.dynamics.BoxShape([0.02] * 3))
    other.createCollisionAspect()
    assert len(link.colliders) == 2
    assert len(set(link.colliders)) == 2
    # Native setup creates both aspects on one node. M5 settles shape/pose
    # observer spelling; both capabilities must follow that same live node.
    node = contact_box.getRootBodyNode().getShapeNode(0)
    node.setShape(dart.dynamics.BoxShape([0.25, 0.2, 0.2]))
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    translation = np.array([0.01, 0.02, 0.03])
    node.setRelativeTransform(dart.math.Isometry3(rotation, translation))
    points = np.eye(3)
    for capability in (collider, visual):
        np.testing.assert_array_equal(capability.shape.size, (0.25, 0.2, 0.2))
        np.testing.assert_allclose(capability.relative_transform @ points, points @ rotation.T + translation)


@pytest.mark.xfail(strict=True, reason="M5b: the final World manual-constraint registry is pending")
def test_manual_constraint_registry(world):
    world.setGravity([0, 0, 0])
    model = dart.dynamics.Skeleton("anchored")
    joint, body = model.createFreeJointAndBodyNodePair()
    world.addSkeleton(model)
    joint.setVelocities([0, 0, 0, 1, 0, 0])
    reference = world.clone()
    anchor = body.getWorldTransform().translation().copy()
    constraint = dart.constraint.BallJointConstraint(body, anchor)
    # Constructor/registry observation spelling is provisional M5b; adding
    # inside edit must retain a constraint that affects the native solve.
    with world.edit():
        world.constraints.add(constraint)
    assert constraint in world.constraints
    for _ in range(20):
        world.step()
        reference.step()
    np.testing.assert_allclose(body.getWorldTransform().translation(), anchor, rtol=0, atol=1e-9)
    free_body = reference.getSkeleton("anchored").getRootBodyNode()
    assert free_body.getWorldTransform().translation()[0] > anchor[0] + 0.01


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
    node = robot.getRootBodyNode().createShapeNode(dart.dynamics.BoxShape([0.2] * 3))
    node.createVisualAspect()
    visual = robot.links["wrist"].visuals[0]
    visual.material = material
    np.testing.assert_array_equal(visual.material.base_color, material.base_color)
    assert visual.material.metallic == material.metallic
    assert visual.material.roughness == material.roughness
    np.testing.assert_array_equal(visual.material.emissive, material.emissive)


@pytest.mark.xfail(
    strict=True, reason="M1g/M5b: joint actuator channels, PositionServo and joint/DOF collections are pending"
)
@pytest.mark.parametrize("target,kd", [(10, 40), (-10, 40), (0.05, 40), (0.05, 0)])
def test_joint_position_servo(world, robot, target, kd):
    world.setGravity([0, 0, 0])
    h = world.getTimeStep()
    inverse_mass = robot.getInvMassMatrix().copy()
    servo = dart.PositionServo(kp=400, kd=kd, effort_limit=80)
    joint = robot.joints["shoulder_yaw"]
    joint.actuator = servo
    robot.dofs.position_targets = np.array([target, 0.0])
    # With zero initial velocity and gravity, only this scalar motor row acts.
    impulse = np.clip(
        h * 400 * target / (1 + (h * kd + h * h * 400) * inverse_mass[0, 0]),
        -h * 80,
        h * 80,
    )
    world.step()
    assert robot.getConstraintForces()[0] == pytest.approx(impulse / h)
    np.testing.assert_allclose(
        robot.getVelocities(), inverse_mass[:, 0] * impulse, rtol=0, atol=1e-12
    )


@pytest.mark.xfail(strict=True, reason="M1g/M5b: VelocityServo and persistent velocity targets are pending")
@pytest.mark.parametrize("target,kd", [(100, 40), (-100, 40), (0.05, 40), (0.05, 0)])
def test_joint_velocity_servo(world, robot, target, kd):
    world.setGravity([0, 0, 0])
    h = world.getTimeStep()
    inverse_mass = robot.getInvMassMatrix().copy()
    # M1g settles constructor and channel spelling. K=0 in the compliant-row
    # equation leaves bounded implicit velocity damping, including zero gain.
    servo = dart.VelocityServo(kd=kd, effort_limit=80)
    robot.joints["shoulder_yaw"].actuator = servo
    robot.dofs.velocity_targets = np.array([target, 0.0])
    impulse = np.clip(
        h * kd * target / (1 + h * kd * inverse_mass[0, 0]), -h * 80, h * 80
    )
    world.step()
    assert robot.getConstraintForces()[0] == pytest.approx(impulse / h)
    np.testing.assert_allclose(
        robot.getVelocities(), inverse_mass[:, 0] * impulse, rtol=0, atol=1e-12
    )


@pytest.mark.xfail(
    strict=True,
    reason="M1h/M5b: native IMUSensor readings and the World sensor registry are pending",
)
def test_imu_sensor(world, robot):
    sensor = dart.sensors.IMUSensor(link=robot.getRootBodyNode())
    imu = world.sensors.add(sensor)
    assert isinstance(imu, dart.sensors.IMUSensor)
    assert imu is sensor
    assert imu in world.sensors
    robot.setPositions(np.full(robot.getNumDofs(), 0.5))
    robot.setVelocities(np.ones(robot.getNumDofs()))
    world.step()
    assert isinstance(imu.orientation, dart.Rotation)
    half = robot.getPositions()[0] / 2
    np.testing.assert_allclose(
        imu.orientation.wxyz, (np.cos(half), np.sin(half), 0, 0), rtol=0, atol=1e-12
    )


@pytest.mark.xfail(strict=True, reason="M1h/M5b: native JointStateSensor registration and samples are pending")
def test_joint_state_sensor(world, robot):
    world.setGravity([0, 0, 0])
    joint = robot.getJoint("shoulder_yaw")
    joint.setPosition(0, 0.2)
    joint.setVelocity(0, 0.7)
    # M1h settles attachment and reading spelling for the named sensor types.
    sensor = dart.sensors.JointStateSensor(joint=joint)
    assert world.sensors.add(sensor) is sensor
    assert sensor in world.sensors
    world.step()
    assert joint.getPosition(0) != 0.2
    assert sensor.position == pytest.approx(joint.getPosition(0))
    assert sensor.velocity == pytest.approx(joint.getVelocity(0))


@pytest.mark.xfail(strict=True, reason="M1h/M5b: native ForceTorqueSensor registration and samples are pending")
def test_force_torque_sensor(world):
    world.setGravity([0, 0, -9.81])
    model = dart.dynamics.Skeleton("supported_body")
    joint, body = model.createWeldJointAndBodyNodePair()
    body.setMass(2)
    world.addSkeleton(model)
    # Attachment and reading spelling are provisional M1h; the support wrench
    # must follow gravity. Magnitude avoids imposing an unstated sign convention.
    sensor = dart.sensors.ForceTorqueSensor(joint=joint)
    assert world.sensors.add(sensor) is sensor
    assert sensor in world.sensors
    for gravity in (9.81, 4.905):
        world.setGravity([0, 0, -gravity])
        world.step()
        assert np.linalg.norm(sensor.force) == pytest.approx(2 * gravity)
        assert np.linalg.norm(sensor.force) == pytest.approx(
            np.linalg.norm(body.getBodyForce()[3:])
        )
        np.testing.assert_allclose(sensor.torque, np.zeros(3), rtol=0, atol=1e-12)


@pytest.mark.xfail(strict=True, reason="M1h/M5b: native ContactSensor registration and samples are pending")
def test_contact_sensor(world, contact_box):
    # Attachment and contact payload spelling are provisional M1h.
    sensor = dart.sensors.ContactSensor(link=contact_box.getRootBodyNode())
    assert world.sensors.add(sensor) is sensor
    assert sensor in world.sensors
    world.step()
    contacts = world.getLastCollisionResult()
    expected = sorted(tuple(contacts.getContact(i).force) for i in range(contacts.getNumContacts()))
    actual = sorted(tuple(contact.force) for contact in sensor.contacts)
    assert actual and any(np.linalg.norm(force) > 0 for force in actual)
    np.testing.assert_array_equal(actual, expected)
    positions = contact_box.getPositions().copy()
    positions[5] = 0.5
    contact_box.setPositions(positions)
    world.step()
    assert world.getLastCollisionResult().getNumContacts() == 0
    assert not sensor.contacts


@pytest.mark.xfail(
    strict=True,
    reason="M1d/M5b: step(n), step_count and rejection of bool arguments are pending",
)
def test_step_n(world, robot):
    world.setGravity([0, 0, 0])
    robot.setVelocities(np.ones(robot.getNumDofs()))
    initial = robot.getPositions().copy()
    serial = world.clone()
    world.step(20)
    assert world.step_count == 20
    for _ in range(20):
        serial.step()
    assert serial.step_count == world.step_count
    assert serial.time == world.time
    assert not np.array_equal(robot.getPositions(), initial)
    serial_robot = serial.getSkeleton("robot")
    np.testing.assert_array_equal(robot.getPositions(), serial_robot.getPositions())
    np.testing.assert_array_equal(robot.getVelocities(), serial_robot.getVelocities())
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

    hook = dart.StepHook(during_step, every=1)
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
        positions = robot.getPositions().copy()
        velocities = robot.getVelocities().copy()
        # The design requires rejection but leaves its exception type to M1d.
        with pytest.raises(Exception):
            current.step()
        assert (current.step_count, current.time) == (count, time)
        np.testing.assert_array_equal(robot.getPositions(), positions)
        np.testing.assert_array_equal(robot.getVelocities(), velocities)

    # Constructor/callback spelling remains provisional M1g.
    world.hooks.add(dart.StepHook(during_step, every=1))
    world.step()
    assert entries == 1
    assert world.step_count == 1


@pytest.mark.xfail(strict=True, reason="M5b: final World lifecycle properties are pending")
def test_world_reset(world, robot, contact_box):
    robot.setVelocities(np.ones(robot.getNumDofs()))
    initial = robot.getPositions().copy()
    for _ in range(3):
        world.step()
    assert not np.array_equal(robot.getPositions(), initial)
    assert world.time > 0 and world.step_count > 0
    assert world.getLastCollisionResult().getNumContacts() > 0
    assert np.linalg.norm(contact_box.getConstraintForces()) > 0
    members = list(world.multibodies)
    states = [(model.getPositions().copy(), model.getVelocities().copy()) for model in (robot, contact_box)]
    # Existing reset semantics clear clocks/history/impulses, preserving q/v.
    # The final design lists reset without redefining it as an initial-state reset.
    world.reset()
    assert world.time == 0 and world.step_count == 0
    assert list(world.multibodies) == members
    assert world.getLastCollisionResult().getNumContacts() == 0
    for model, (positions, velocities) in zip((robot, contact_box), states):
        np.testing.assert_array_equal(model.getPositions(), positions)
        np.testing.assert_array_equal(model.getVelocities(), velocities)
        np.testing.assert_array_equal(model.getConstraintForces(), np.zeros(model.getNumDofs()))
    world.step()
    assert world.step_count == 1
    assert world.time == pytest.approx(world.getTimeStep())
    assert not np.array_equal(robot.getPositions(), states[0][0])


@pytest.mark.xfail(strict=True, reason="M1g/M5b: StepHook and the World hook registry are pending")
def test_step_hook_every(world, robot):
    calls = []

    # M1g settles the callback signature; decimation and global boundaries are
    # the acceptance contract regardless of the final registration spelling.
    hook = dart.StepHook(lambda world: calls.append(world.step_count), every=3)
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
    expected = np.full(robot.getNumDofs(), 0.5)
    setattr(robot.dofs, channel, expected)
    array = getattr(robot.dofs, channel)
    assert not array.flags.writeable
    with pytest.raises(ValueError):
        array[0] = -0.5
    np.testing.assert_array_equal(getattr(robot.dofs, channel), expected)
    # Changing native storage must not mutate an already returned copy.
    setattr(robot.dofs, channel, -expected)
    np.testing.assert_array_equal(array, expected)


@pytest.mark.xfail(strict=True, reason="M5: immutable Transform composition is pending")
def test_transform_composition():
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


@pytest.mark.xfail(
    strict=True,
    reason="M5: links, Link.world_transform and immutable Transform are pending",
)
def test_link_world_transform(robot):
    # The x-axis shoulder keeps the rotation from commuting with the z offset,
    # so a reversed composition order also fails.
    robot.setPositions(np.full(robot.getNumDofs(), 0.5))
    X_WP = robot.links["palm"].world_transform @ dart.Transform(translation=(0, 0, 0.1))
    X_WL = robot.getBodyNode("palm").getWorldTransform().matrix()
    points = np.eye(3)
    expected = (points + (0, 0, 0.1)) @ X_WL[:3, :3].T + X_WL[:3, 3]
    np.testing.assert_allclose(X_WP @ points, expected, rtol=0, atol=1e-12)


@pytest.mark.xfail(
    strict=True,
    reason="M1c/M1g/M5b: dynamic snapshots including persistent servo targets are pending",
)
def test_capture_restore_state(world, robot):
    robot.joints["shoulder_yaw"].actuator = dart.PositionServo(
        kp=400, kd=40, effort_limit=80
    )
    targets = np.full(robot.getNumDofs(), 0.5)
    velocity_targets = np.full(robot.getNumDofs(), 0.25)
    robot.dofs.position_targets = targets
    robot.dofs.velocity_targets = velocity_targets
    robot.setVelocities(np.ones(robot.getNumDofs()))
    positions = robot.getPositions().copy()
    velocities = robot.getVelocities().copy()
    time = world.time
    state = world.capture_state()
    clone = world.clone()
    clone_robot = clone.getSkeleton("robot")
    clone.setGravity([0, 0, 0])
    clone_robot.setPositions(-positions + 0.2)
    clone_robot.setVelocities(-velocities)
    clone_robot.dofs.position_targets = -targets
    clone_robot.dofs.velocity_targets = -velocity_targets
    clone.restore_state(state)
    np.testing.assert_array_equal(clone.gravity, (0, 0, 0))
    np.testing.assert_array_equal(clone_robot.getPositions(), positions)
    np.testing.assert_array_equal(clone_robot.getVelocities(), velocities)
    np.testing.assert_array_equal(clone_robot.dofs.position_targets, targets)
    np.testing.assert_array_equal(clone_robot.dofs.velocity_targets, velocity_targets)
    assert clone.time == time
    # Equal array sizes do not imply a matching structural digest.
    different = world.clone()
    different_robot = different.getSkeleton("robot")
    with different.edit():
        fixed = dart.dynamics.Skeleton("extra_fixed_model")
        fixed.createWeldJointAndBodyNodePair()
        different.addSkeleton(fixed)
    assert fixed.getNumDofs() == 0
    different_robot.setPositions(np.full(robot.getNumDofs(), 0.25))
    different_robot.setVelocities(-velocities)
    different_robot.dofs.position_targets = -targets
    different_robot.dofs.velocity_targets = -velocity_targets
    before = (
        different_robot.getPositions().tobytes(),
        different_robot.getVelocities().tobytes(),
        different_robot.dofs.position_targets.tobytes(),
        different_robot.dofs.velocity_targets.tobytes(),
        different.time,
    )
    with pytest.raises(Exception):
        different.restore_state(state)
    assert (
        different_robot.getPositions().tobytes(),
        different_robot.getVelocities().tobytes(),
        different_robot.dofs.position_targets.tobytes(),
        different_robot.dofs.velocity_targets.tobytes(),
        different.time,
    ) == before
    robot.dofs.position_targets = np.full(robot.getNumDofs(), -0.5)
    robot.dofs.velocity_targets = -velocity_targets
    np.testing.assert_array_equal(robot.dofs.position_targets, -targets)
    world.step()
    assert not np.array_equal(robot.getPositions(), positions)
    world.restore_state(state)
    np.testing.assert_array_equal(robot.getPositions(), positions)
    np.testing.assert_array_equal(robot.getVelocities(), velocities)
    np.testing.assert_array_equal(robot.dofs.position_targets, targets)
    np.testing.assert_array_equal(robot.dofs.velocity_targets, velocity_targets)
    assert world.time == time


@pytest.mark.xfail(
    strict=True, reason="M1c/M5b: exact checkpoints, bit-identical replay and mismatch rejection are pending"
)
def test_capture_restore_checkpoint(world, contact_box):
    robot = contact_box
    options = world.getDeactivationOptions()
    options.mEnabled = True
    options.mTimeUntilSleep = 5 * world.getTimeStep()
    world.setDeactivationOptions(options)
    for _ in range(2):
        world.step()
    assert not robot.isResting()
    checkpoint = world.capture_checkpoint()

    def frame():
        contacts = world.getLastCollisionResult()
        contact_state = tuple(
            (
                contact.point.tobytes(),
                contact.normal.tobytes(),
                contact.force.tobytes(),
                contact.penetrationDepth,
            )
            for contact in (contacts.getContact(i) for i in range(contacts.getNumContacts()))
        )
        # Serialized fields plus solver outputs and sleep transitions are the oracle.
        return (
            pickle.dumps(world.capture_checkpoint(), protocol=5),
            world.time,
            world.step_count,
            robot.getPositions().tobytes(),
            robot.getVelocities().tobytes(),
            robot.getAccelerations().tobytes(),
            robot.getForces().tobytes(),
            (robot.getConstraintForces() * world.getTimeStep()).tobytes(),
            contact_state,
            robot.isResting(),
        )

    initial = frame()
    expected = []
    for _ in range(20):
        world.step()
        expected.append(frame())
    assert robot.isResting()
    assert expected[-1] != initial
    world.restore_state(pickle.loads(pickle.dumps(checkpoint, protocol=5)))
    assert frame() == initial
    for expected_frame in expected:
        world.step()
        assert frame() == expected_frame

    world.setGravity([0, 0, 0])
    before_rejection = frame()
    with pytest.raises(Exception) as mismatch:
        world.restore_state(checkpoint)
    assert type(mismatch.value).__name__ == "CheckpointMismatchError"
    assert frame() == before_rejection


@pytest.fixture
def contact_box(world):
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


def test_checkpoint_contact_scene(world, contact_box):
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


@pytest.mark.xfail(
    strict=True, reason="M1i/M1g/M5b: WorldBatch clones, set/step and the position-target channel are pending"
)
def test_world_batch_set_step(world, robot):
    world.setGravity([0, 0, 0])
    robot.setVelocities(np.zeros(robot.getNumDofs()))
    servo = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    robot.joints["shoulder_yaw"].actuator = servo
    batch = dart.WorldBatch(world, count=64)
    assert batch.positions.shape == (64, robot.getNumDofs())
    batch.set("velocities", np.zeros_like(batch.positions))
    targets = np.zeros_like(batch.positions)
    targets[:, 0] = np.linspace(-0.5, 0.5, 64)
    batch.set("position_targets", targets)
    batch.step(20)
    expected = []
    for target in targets:
        serial = world.clone()
        serial_robot = serial.getSkeleton("robot")
        serial_robot.dofs.position_targets = target
        for _ in range(20):
            serial.step()
        expected.append(serial_robot.getPositions().copy())
    assert expected[0][0] < 0 < expected[-1][0]
    np.testing.assert_array_equal(batch.positions, expected)
    # The source is a template; stepping its independent clones leaves it alone.
    np.testing.assert_array_equal(robot.getPositions(), np.zeros(robot.getNumDofs()))


@pytest.mark.xfail(strict=True, reason="M1c/M1i: selected-row WorldBatch snapshot broadcasting is pending")
def test_world_batch_restore_state(world, robot):
    robot.setPositions([0.2, -0.3])
    robot.setVelocities([0.4, -0.5])
    source_positions = robot.getPositions().copy()
    source_velocities = robot.getVelocities().copy()
    state = world.capture_state()
    batch = dart.WorldBatch(world, count=4)
    positions = np.array([[1, 2], [3, 4], [5, 6], [7, 8]], dtype=float)
    velocities = -positions
    batch.set("positions", positions)
    batch.set("velocities", velocities)
    # The array observer spelling is provisional M1i. The feature PR must
    # also cover control channels and each clone's clock.
    before_positions = batch.positions.copy()
    before_velocities = batch.velocities.copy()
    np.testing.assert_array_equal(before_positions, positions)
    np.testing.assert_array_equal(before_velocities, velocities)
    batch.restore_state(state, indices=[0, 2])
    expected_positions = before_positions.copy()
    expected_velocities = before_velocities.copy()
    expected_positions[[0, 2]] = source_positions
    expected_velocities[[0, 2]] = source_velocities
    np.testing.assert_array_equal(batch.positions, expected_positions)
    np.testing.assert_array_equal(batch.velocities, expected_velocities)
    np.testing.assert_array_equal(robot.getPositions(), source_positions)
    np.testing.assert_array_equal(robot.getVelocities(), source_velocities)


@pytest.mark.xfail(strict=True, reason="M1i/M1g/M5b: remapping clones with World-bound step hooks are pending")
def test_world_batch_remaps_hook(world, robot):
    world.setGravity([0, 0, 0])
    robot.joints["shoulder_yaw"].actuator = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    robot.dofs.position_targets = np.zeros(robot.getNumDofs())

    def drive(current):
        # Stateless and bound through the callback's World, not a source model
        # captured in a closure. M1g settles the callback signature.
        current.getSkeleton("robot").dofs.position_targets = np.array([0.5, 0])

    world.hooks.add(dart.StepHook(drive, every=1))
    before_time = world.time
    batch = dart.WorldBatch(world, count=3)
    batch.step(20)
    assert world.time == before_time
    np.testing.assert_array_equal(robot.getPositions(), np.zeros(robot.getNumDofs()))
    np.testing.assert_array_equal(robot.dofs.position_targets, np.zeros(robot.getNumDofs()))
    # Step the actual source as oracle: a shallow clone that drops the hook
    # cannot make both the batch and a cloned serial oracle fail identically.
    for _ in range(20):
        world.step()
    assert robot.getPositions()[0] > 0
    np.testing.assert_array_equal(batch.positions, np.tile(robot.getPositions(), (3, 1)))
