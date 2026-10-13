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
def test_joint_position_servo(robot):
    servo = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    robot.joints["shoulder_yaw"].actuator = servo
    robot.dofs.position_targets = np.zeros(robot.getNumDofs())
    assert isinstance(robot.joints["shoulder_yaw"].actuator, dart.PositionServo)


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
def test_step_n(world):
    world.step(20)
    assert world.step_count == 20
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
def test_capture_restore_checkpoint(world, robot):
    world.getConstraintSolver().setCollisionDetector(
        dart.collision.DARTCollisionDetector()
    )
    robot.setVelocities(np.ones(robot.getNumDofs()))
    world.step()
    checkpoint = world.capture_checkpoint()

    def frame():
        # Compare serialized fields and bit-exact public state, not native memory.
        return (
            pickle.dumps(world.capture_checkpoint(), protocol=5),
            world.time,
            world.step_count,
            robot.getPositions().tobytes(),
            robot.getVelocities().tobytes(),
            robot.getAccelerations().tobytes(),
            robot.getForces().tobytes(),
        )

    initial = frame()
    expected = []
    for _ in range(10):
        world.step()
        expected.append(frame())
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
    batch.set("position_targets", np.full_like(batch.positions, 0.5))
    batch.step(20)
    robot.dofs.position_targets = np.full(robot.getNumDofs(), 0.5)
    for _ in range(20):
        world.step()
    assert robot.getPositions()[0] > 0
    np.testing.assert_array_equal(
        batch.positions, np.tile(robot.getPositions(), (64, 1))
    )
