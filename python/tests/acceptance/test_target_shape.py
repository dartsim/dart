"""DART 7.0 target shape; each feature PR removes its strict xfail marker.

The PR that removes a marker must make its test exercise the full contract in
docs/design/dart7_python_api.md.

The headless-render part needs a graphics device, so it lives in
python/tests/unit/gui/test_target_shape_render.py, which test-gui runs and
test-py leaves out.
"""

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


@pytest.mark.xfail(
    strict=True, reason="M1b/M5b: edit/compile modes and frozen topology enforcement are pending"
)
def test_edit_compile(world, robot):
    world.compile()
    assert world.is_compiled
    # The design names the error but leaves its public module to M5.
    with pytest.raises(Exception) as frozen:
        world.remove(robot)
    assert type(frozen.value).__name__ == "ModelFrozenError"
    assert robot in world.multibodies
    with world.edit():
        assert not world.is_compiled
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
@pytest.mark.parametrize("channel", ["positions", "velocities", "position_targets"])
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
    robot.dofs.position_targets = targets
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
    clone.restore_state(state)
    np.testing.assert_array_equal(clone_robot.getPositions(), positions)
    np.testing.assert_array_equal(clone_robot.getVelocities(), velocities)
    np.testing.assert_array_equal(clone_robot.dofs.position_targets, targets)
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
    before = (
        different_robot.getPositions().tobytes(),
        different_robot.getVelocities().tobytes(),
        different_robot.dofs.position_targets.tobytes(),
        different.time,
    )
    with pytest.raises(Exception):
        different.restore_state(state)
    assert (
        different_robot.getPositions().tobytes(),
        different_robot.getVelocities().tobytes(),
        different_robot.dofs.position_targets.tobytes(),
        different.time,
    ) == before
    robot.dofs.position_targets = np.full(robot.getNumDofs(), -0.5)
    np.testing.assert_array_equal(robot.dofs.position_targets, -targets)
    world.step()
    assert not np.array_equal(robot.getPositions(), positions)
    world.restore_state(state)
    np.testing.assert_array_equal(robot.getPositions(), positions)
    np.testing.assert_array_equal(robot.getVelocities(), velocities)
    np.testing.assert_array_equal(robot.dofs.position_targets, targets)
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
