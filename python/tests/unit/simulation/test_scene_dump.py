from __future__ import annotations

import math
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[4]


class _FakeShapeType:
    def __init__(self, name: str) -> None:
        self.name = name


class _FakeShape:
    def __init__(self, name: str, **attrs) -> None:
        self.type = _FakeShapeType(name)
        self.local_transform = np.eye(4)
        for key, value in attrs.items():
            setattr(self, key, value)


def _simulation():
    if not hasattr(dart, "World"):
        raise AssertionError("dartpy imported but did not expose World")
    return dart


def test_compute_step_metrics_binding_reports_finite_step_state():
    sx = _simulation()

    world = sx.World(time_step=0.005)
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.5))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((2.0, 2.0, 0.5)))

    sphere = world.add_rigid_body("sphere", mass=2.0, position=(0.0, 0.0, 1.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.5))

    world.step(n=120)

    metrics = world.compute_step_metrics()
    assert isinstance(metrics, sx.StepMetrics)
    assert math.isfinite(metrics.kinetic_energy)
    assert math.isfinite(metrics.potential_energy)
    assert math.isfinite(metrics.total_energy)
    assert np.isfinite(np.asarray(metrics.linear_momentum, dtype=float)).all()
    assert np.isfinite(np.asarray(metrics.angular_momentum, dtype=float)).all()
    # The sphere has settled into resting contact on the ground box after 120
    # steps, so these must reflect real physics -- a stubbed binding returning
    # all zeros would pass finiteness but fail here.
    assert metrics.active_contact_count >= 1
    assert abs(metrics.potential_energy) > 0.0
    assert math.isfinite(metrics.max_penetration_depth)
    assert metrics.max_penetration_depth >= 0.0
    assert metrics.last_step_iterations >= 0
    assert math.isfinite(metrics.last_step_residual)


def test_agent_verification_symbols_are_in_committed_stubs():
    root_stub = (ROOT / "python" / "stubs" / "dartpy" / "__init__.pyi").read_text()
    simulation_stub = (
        ROOT / "python" / "stubs" / "dartpy" / "simulation.pyi"
    ).read_text()

    for token in (
        "StepMetrics",
        "def compute_step_metrics(self) -> StepMetrics",
        "def dump_scene_json(world: Any) -> dict[str, Any]",
        "def dump_scene_text(world: Any) -> str",
        "MultibodyIntegrationFamily",
    ):
        assert token in simulation_stub

    for token in (
        "StepMetrics",
        "dump_scene_json",
        "dump_scene_text",
        "MultibodyIntegrationFamily",
    ):
        assert token in root_stub


def test_scene_dump_json_and_text_match_agent_schema():
    sx = _simulation()

    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, -9.81))
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.08))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((5.0, 5.0, 0.08)))

    box = world.add_rigid_body(
        "box",
        mass=2.0,
        position=(0.0, 0.0, 0.5),
        linear_velocity=(0.1, 0.0, 0.0),
        inertia=np.diag([0.01, 0.02, 0.03]),
    )
    box.set_collision_shape(sx.CollisionShape.box((0.1, 0.2, 0.3)))

    joint = world.add_joint(
        ground,
        box,
        sx.JointSpec(
            name="box_slider",
            type=sx.JointType.PRISMATIC,
            axis=(1.0, 0.0, 0.0),
        ),
    )
    joint.position = [0.25]
    joint.set_position_limits([-0.5], [0.5])

    scene = sx.dump_scene_json(world)

    assert scene["schema"] == "dart.scene/v0"
    assert set(scene) == {"schema", "world", "bodies", "joints", "flat_index"}
    assert scene["world"]["gravity"] == pytest.approx([0.0, 0.0, -9.81])
    assert scene["world"]["time_step"] == pytest.approx(0.005)
    assert scene["world"]["units"] == {
        "length": "meter",
        "mass": "kilogram",
        "time": "second",
        "angle": "radian",
    }
    assert "rest_tolerance" in scene["world"]
    assert scene["world"]["body_count"] == 2
    assert scene["world"]["joint_count"] == 1

    bodies = {body["name"]: body for body in scene["bodies"]}
    assert set(bodies) == {"ground", "box"}
    assert bodies["ground"]["kind"] == "rigid"
    assert bodies["ground"]["is_static"] is True
    assert bodies["ground"]["collision_shape"]["type"] == "box"
    assert bodies["ground"]["collision_shape"]["half_extents"] == pytest.approx(
        [5.0, 5.0, 0.08]
    )

    box_dump = bodies["box"]
    assert box_dump["mass"] == pytest.approx(2.0)
    assert box_dump["position"] == pytest.approx([0.0, 0.0, 0.5])
    assert box_dump["linear_velocity"] == pytest.approx([0.1, 0.0, 0.0])
    assert box_dump["inertia_diag"] == pytest.approx([0.01, 0.02, 0.03])
    assert box_dump["has_collision_shape"] is True
    assert len(box_dump["shapes"]) == 1
    assert box_dump["shapes"][0]["type"] == "box"
    assert box_dump["shapes"][0]["size"] == pytest.approx([0.2, 0.4, 0.6])
    assert "local_transform" in box_dump["shapes"][0]

    assert len(scene["joints"]) == 1
    joint_dump = scene["joints"][0]
    assert joint_dump["name"] == "box_slider"
    assert joint_dump["type"] == "prismatic"
    assert joint_dump["position"] == pytest.approx([0.25])
    assert joint_dump["limits"]["lower"] == pytest.approx([-0.5])
    assert joint_dump["limits"]["upper"] == pytest.approx([0.5])

    assert "world" in scene["flat_index"]
    assert "body:ground" in scene["flat_index"]
    assert "body:box" in scene["flat_index"]
    assert "shape:body:box:0" in scene["flat_index"]
    assert "joint:box_slider" in scene["flat_index"]

    text = sx.dump_scene_text(world)
    assert text.startswith("# dart.scene/v0")
    assert "gravity=[0, 0, -9.81]" in text
    assert "body:box rigid body box" in text
    assert "joint:box_slider prismatic joint box_slider" in text
    assert "flat_index:" in text


def test_capsule_scene_dump_size_includes_caps():
    sx = _simulation()

    world = sx.World()
    body = world.add_rigid_body("cap", position=(0.0, 0.0, 1.0))
    body.set_collision_shape(sx.CollisionShape.capsule(0.3, 0.5))

    scene = sx.dump_scene_json(world)
    shape = {b["name"]: b for b in scene["bodies"]}["cap"]["collision_shape"]
    assert shape["type"] == "capsule"
    assert shape["radius"] == pytest.approx(0.3)
    assert shape["half_height"] == pytest.approx(0.5)
    # Full axial extent includes both hemispherical caps: 2*half_height + 2*radius
    # (not the bare cylinder segment 2*half_height).
    assert shape["size"] == pytest.approx([0.6, 0.6, 1.6])


def test_scene_dump_serializes_followup_shape_fields():
    from dartpy import _scene_dump

    cone = _scene_dump._shape_to_json(
        _FakeShape("CONE", radius=0.4, height=1.5),
        "body:cone",
        0,
    )
    assert cone["type"] == "cone"
    assert cone["size"] == pytest.approx([0.8, 0.8, 1.5])
    assert cone["radius"] == pytest.approx(0.4)
    assert cone["height"] == pytest.approx(1.5)

    heightmap = _scene_dump._shape_to_json(
        _FakeShape(
            "HEIGHTMAP",
            size=(2.0, 3.0, 0.5),
            heights=np.array([[0.0, 0.25], [0.5, -0.25]]),
            width=2.0,
            depth=3.0,
        ),
        "body:heightmap",
        0,
    )
    assert heightmap["type"] == "heightmap"
    assert heightmap["size"] == pytest.approx([2.0, 3.0, 0.5])
    assert heightmap["height_sample_count"] == 4
    assert heightmap["height_range"] == pytest.approx([-0.25, 0.5])

    soft_body = _scene_dump._shape_to_json(
        _FakeShape(
            "SOFT_BODY",
            vertices=[(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)],
            triangles=[(0, 1, 1)],
            tetrahedra=[(0, 1, 1, 1)],
        ),
        "body:soft",
        0,
    )
    assert soft_body["type"] == "soft_body"
    assert soft_body["size"] == pytest.approx([1.0, 2.0, 3.0])
    assert soft_body["vertex_count"] == 2
    assert soft_body["triangle_count"] == 1
    assert soft_body["tetrahedron_count"] == 1
