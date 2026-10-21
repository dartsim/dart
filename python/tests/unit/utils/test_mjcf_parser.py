import os
import platform

import numpy as np

import dartpy as dart
import pytest
from tests.util import get_asset_path


def test_parse_fetch():
    world = dart.utils.MjcfParser.readWorld(
        "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml"
    )
    assert world is not None
    # M1f switches only the Python World and io.load_world wrappers. The
    # direct binding calls the C++ parser, whose FCL default must stay intact.
    assert isinstance(
        world.getConstraintSolver().getCollisionDetector(),
        dart.collision.FCLCollisionDetector,
    )


@pytest.mark.parametrize(
    "name,limited",
    [
        ("hinge_true", True),
        ("slide_true", True),
        ("false_range", False),
        ("false_no_range", False),
        ("auto_range", True),
        ("auto_no_range", False),
        ("auto_zero", False),
        ("auto_reversed", False),
        ("implicit_auto", True),
        ("inherited_true", True),
        ("inherited_child", True),
        ("inherited_false", False),
        ("inherited_auto", True),
        ("inherited_zero", False),
        ("overridden_false", False),
        ("stacked_hinge", True),
        ("stacked_slide", True),
        ("stacked_false_hinge", False),
        ("stacked_false_slide", False),
        ("stacked_limited_slide", True),
        ("ball_true", False),
        ("ball_no_range", False),
    ],
)
def test_joint_limits(name, limited):
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/test/joint_limits.xml")
    assert world is not None
    joint = world.getSkeleton("limits").getJoint(name)
    assert joint.areLimitsEnforced() == limited
    if limited:
        expected = [-0.1, 0.2] if "slide" in name else np.deg2rad([-30, 60])
        np.testing.assert_allclose(
            [joint.getPositionLowerLimit(0), joint.getPositionUpperLimit(0)], expected
        )
    else:
        for i in range(joint.getNumDofs()):
            assert joint.getPositionLowerLimit(i) == -np.inf
            assert joint.getPositionUpperLimit(i) == np.inf


def test_autolimits_disabled():
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/test/autolimits_false.xml")
    assert world is not None
    skeleton = world.getSkeleton("explicit_true")
    joint = skeleton.getRootJoint()
    assert joint.areLimitsEnforced()
    assert joint.getPositionLowerLimit(0) == -0.3
    assert joint.getPositionUpperLimit(0) == 0.6
    assert joint.getRestPosition(0) == 0.3
    for name in ["explicit_false", "auto_zero", "inherited_zero"]:
        joint = skeleton.getJoint(name)
        assert not joint.areLimitsEnforced()
        assert joint.getPositionLowerLimit(0) == -np.inf
        assert joint.getPositionUpperLimit(0) == np.inf


def test_multislide_limits():
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/test/joint_limits.xml")
    assert world is not None
    skeleton = world.getSkeleton("limits")
    for name in ["slides2", "slides3"]:
        joint = skeleton.getBodyNode(name).getParentJoint()
        assert joint.areLimitsEnforced()
        last = joint.getNumDofs() - 1
        for i in range(last):
            assert joint.getPositionLowerLimit(i) == -np.inf
            assert joint.getPositionUpperLimit(i) == np.inf
        assert joint.getPositionLowerLimit(last) == -0.1
        assert joint.getPositionUpperLimit(last) == 0.2


def test_spring_reference_angle_units():
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/test/joint_limits.xml")
    assert world is not None
    skeleton = world.getSkeleton("limits")
    assert skeleton.getJoint("hinge_true").getRestPosition(0) == pytest.approx(np.pi / 6)
    assert skeleton.getJoint("slide_true").getRestPosition(0) == 0.1


@pytest.mark.parametrize(
    "compiler,attributes",
    [
        ("", "limited='invalid'"),
        ("", "limited='True'"),
        ("", "limited='true'"),
        ("", "limited='true' range='1 -1'"),
        ("", "limited='true' range='nan 1'"),
        ("", "limited='true' range='-1 inf'"),
        ("", "type='ball' limited='true' range='1 30'"),
        ("", "type='ball' limited='auto' range='1 30'"),
        ("", "limited='auto' range='-1 inf'"),
        ("autolimits='false'", "range='-1 1'"),
        ("autolimits='false'", "limited='auto' range='-1 1'"),
        ("autolimits='false'", "limited='auto' range='1 -1'"),
    ],
)
def test_invalid_joint_limits(tmp_path, compiler, attributes):
    model = tmp_path / "limits.xml"
    model.write_text(
        f"<mujoco><compiler {compiler}/><worldbody><body>"
        f"<joint {attributes}/>"
        "<inertial pos='0 0 0' mass='1' diaginertia='2 3 4'/>"
        "</body></worldbody></mujoco>"
    )
    assert dart.utils.MjcfParser.readWorld(str(model)) is None


@pytest.mark.parametrize("value", ["invalid", "True", "False", "1"])
def test_invalid_autolimits(tmp_path, value):
    model = tmp_path / "autolimits.xml"
    model.write_text(f"<mujoco><compiler autolimits='{value}'/></mujoco>")
    assert dart.utils.MjcfParser.readWorld(str(model)) is None


@pytest.mark.parametrize(
    "name,expected",
    [
        (name, [[2.5, -0.5, 0], [-0.5, 2.5, 0], [0, 0, 4]])
        for name in ["quat", "axisangle", "euler", "xyaxes", "no_body_pos"]
    ]
    + [
        ("zaxis", np.diag([4, 3, 2])),
        ("full", [[3, 0.2, -0.3], [0.2, 4, 0.4], [-0.3, 0.4, 5]]),
    ],
)
def test_declared_inertia(name, expected):
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/test/declared_inertia.xml")
    assert world is not None
    body = world.getSkeleton("quat").getBodyNode(name)
    np.testing.assert_allclose(body.getInertia().getMoment(), expected, atol=1e-12)
    assert body.getMass() == (3 if name.startswith("full") else 2)
    expected_com = [-0.1, 0.2, -0.3] if name.startswith("full") else [0.1, 0.2, 0.3]
    np.testing.assert_allclose(body.getLocalCOM(), expected_com)


def test_omitted_body_position_and_orthogonalized_xyaxes():
    world = dart.utils.MjcfParser.readWorld("dart://sample/mjcf/test/declared_inertia.xml")
    assert world is not None
    body = world.getSkeleton("quat").getBodyNode("no_inertial")
    np.testing.assert_allclose(body.getRelativeTransform().translation(), [0, 0, 0])
    np.testing.assert_allclose(
        body.getWorldTransform().translation(),
        body.getParentBodyNode().getWorldTransform().translation(),
    )
    assert body.getMass() == 7
    np.testing.assert_allclose(body.getLocalCOM(), [0.1, 0.2, 0.3])
    c = np.sqrt(0.5)
    expected = [[c, -c, 0], [c, c, 0], [0, 0, 1]]
    np.testing.assert_allclose(body.getRelativeTransform().rotation(), expected, atol=1e-12)
    assert body.getNumShapeNodes() == 2
    for i in range(body.getNumShapeNodes()):
        rotation = body.getShapeNode(i).getRelativeTransform().rotation()
        np.testing.assert_allclose(rotation, expected, atol=1e-12)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)


@pytest.mark.parametrize(
    "orientation",
    [
        "quat='1 0 0 0'",
        "axisangle='0 0 1 0'",
        "euler='0 0 0'",
        "xyaxes='1 0 0 0 1 0'",
        "zaxis='0 0 1'",
    ],
)
def test_fullinertia_conflicts_with_orientation(tmp_path, orientation):
    model = tmp_path / "inertia.xml"
    model.write_text(
        "<mujoco><worldbody><body><inertial pos='0 0 0' mass='1' "
        f"fullinertia='3 4 5 0.2 -0.3 0.4' {orientation}/>"
        "</body></worldbody></mujoco>"
    )
    assert dart.utils.MjcfParser.readWorld(str(model)) is None


if __name__ == "__main__":
    pytest.main()
