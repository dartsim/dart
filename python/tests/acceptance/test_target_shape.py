"""DART 7.0 target shape; each feature PR removes its strict xfail marker."""

import dartpy as dart
import numpy as np
import pytest


@pytest.fixture
def world():
    # Existing constructors keep each failure local to the feature under test.
    return dart.simulation.World()


@pytest.fixture
def robot(world):
    robot = dart.dynamics.Skeleton("robot")
    joint, link = robot.createRevoluteJointAndBodyNodePair()
    joint.setName("shoulder_yaw")
    link.setName("wrist")
    world.addSkeleton(robot)
    return robot


@pytest.mark.xfail(
    strict=True, reason="M5: top-level World and keyword-only construction are pending"
)
def test_world_construction_keywords():
    world = dart.World(time_step=1e-3, gravity=(0, 0, -9.81))
    assert world.time_step == 1e-3
    np.testing.assert_array_equal(world.gravity, (0, 0, -9.81))


@pytest.mark.xfail(strict=True, reason="M1b: explicit edit/compile modes are pending")
def test_edit_compile(world):
    world.compile()
    assert world.is_compiled
    with world.edit():
        pass
    assert world.is_compiled


@pytest.mark.xfail(strict=True, reason="M5: io.load_model and dart.data are pending")
def test_load_model_and_data():
    path = dart.data.path("urdf/KR5/KR5 sixx R650.urdf")
    robot = dart.io.load_model(path)
    assert robot.num_dofs == 6


@pytest.mark.xfail(
    strict=True, reason="M1g/M5b: joint actuator channels and PositionServo are pending"
)
def test_joint_position_servo(robot):
    servo = dart.PositionServo(kp=400, kd=40, effort_limit=80)
    robot.joints["shoulder_yaw"].actuator = servo
    robot.dofs.position_targets = np.zeros(robot.getNumDofs())
    assert isinstance(robot.joints["shoulder_yaw"].actuator, dart.PositionServo)


@pytest.mark.xfail(
    strict=True,
    reason="M1h/M5b: native IMUSensor and the World sensor registry are pending",
)
def test_imu_sensor(world, robot):
    sensor = dart.sensors.IMUSensor(link=robot.getRootBodyNode())
    imu = world.sensors.add(sensor)
    assert isinstance(imu, dart.sensors.IMUSensor)


@pytest.mark.xfail(
    strict=True,
    reason="M1d/M5b: step(n) is pending; the current argument accepts only bool",
)
def test_step_n(world):
    world.step(20)
    assert world.getSimFrames() == 20


@pytest.mark.xfail(strict=True, reason="M5: immutable Transform composition is pending")
def test_transform_composition():
    parent = dart.Transform(translation=(1, 2, 3))
    local = dart.Transform(translation=(0, 0, 0.1))
    composed = parent @ local
    np.testing.assert_allclose(composed @ np.zeros(3), (1, 2, 3.1))


@pytest.mark.xfail(
    strict=True,
    reason="M1c/M5b: capture_state/restore_state dynamic snapshots are pending",
)
def test_capture_restore_state(world, robot):
    robot.setVelocities(np.ones(robot.getNumDofs()))
    positions = robot.getPositions().copy()
    velocities = robot.getVelocities().copy()
    time = world.getTime()
    state = world.capture_state()
    world.step()
    assert not np.array_equal(robot.getPositions(), positions)
    world.restore_state(state)
    np.testing.assert_array_equal(robot.getPositions(), positions)
    np.testing.assert_array_equal(robot.getVelocities(), velocities)
    assert world.getTime() == time


@pytest.mark.xfail(
    strict=True, reason="M6b: dart.render.Renderer and Camera RGB capture are pending"
)
def test_renderer_rgb(world):
    rgb = dart.render.Renderer(world).render(
        dart.render.Camera.look_at(eye=(2, 0, 1), target=(0, 0, 0.5))
    )
    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8


@pytest.mark.xfail(
    strict=True, reason="M1i/M5b: WorldBatch remapping clones and set/step are pending"
)
def test_world_batch_set_step(world, robot):
    batch = dart.WorldBatch(world, count=64)
    assert batch.positions.shape == (64, robot.getNumDofs())
    batch.set("velocities", np.ones_like(batch.positions))
    batch.set("position_targets", np.zeros_like(batch.positions))
    batch.step(20)
    robot.setVelocities(np.ones(robot.getNumDofs()))
    for _ in range(20):
        world.step()
    assert np.any(robot.getPositions() != 0)
    np.testing.assert_array_equal(
        batch.positions, np.tile(robot.getPositions(), (64, 1))
    )
