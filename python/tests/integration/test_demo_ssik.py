"""The ssik catalog controls and the former 19-arm example's self-test."""

import importlib
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
from examples.demos.models.arm_skeleton import (
    ArmSkeleton,
    get_ssik_arm_specs,
    make_ssik_target_transform,
    ssik_target_controls,
)
viewer = getattr(dart, "viewer", None)
requires_viewer = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")


def _scene_module():
    from examples.demos.scenes import ssik_ik_gui

    return ssik_ik_gui


def _edit(control, value=None):
    control.edit(value)
    if control.on_change:
        control.on_change()


@pytest.fixture
def controls(monkeypatch):
    recorded = []
    for name in (
        "add_text",
        "add_choice",
        "add_vec3",
        "add_slider",
        "add_checkbox",
        "add_button",
    ):
        original = getattr(viewer.Control, name)

        def record(self, *args, _original=original, **kwargs):
            control = _original(self, *args, **kwargs)
            recorded.append(control)
            return control

        monkeypatch.setattr(viewer.Control, name, record)
    return recorded


def _control(controls, label):
    return next(control for control in controls if control.label == label)


class _Context:
    def __init__(self):
        self.attachments = []
        self.drag = []

    def add_attachment(self, attachment):
        self.attachments.append(attachment)

    def enable_drag_and_drop(self, frame):
        self.drag.append(frame)


@pytest.mark.parametrize("arm_index", range(19))
def test_ssik_arm_fk_and_python_analytical_targets(arm_index):
    """Keep both independent checks from the removed example's self-test."""
    pytest.importorskip("ssik")
    scene_module = _scene_module()
    specs = get_ssik_arm_specs()
    assert len(specs) == 19
    module = importlib.import_module("ssik.prebuilt." + specs[arm_index].module)
    world = dart.simulation.World()
    arm = ArmSkeleton(world)
    arm.build(scene_module.arm_chain(module))
    arm.show_solutions([], -1)
    skeleton = world.getSkeleton(0)
    ee = skeleton.getBodyNode(skeleton.getNumBodyNodes() - 1)
    dof = skeleton.getNumDofs()
    assert dof == module.DOF
    q = 0.1 * np.cos(np.arange(dof))
    skeleton.setPositions(q)
    assert (
        np.linalg.norm(
            ee.getWorldTransform().matrix() - np.asarray(module.fk(q)).reshape((4, 4))
        )
        < 1e-6
    )

    respect_limits = True

    def solve(target):
        return list(
            module.solve(
                np.asarray(target).reshape((4, 4)),
                max_solutions=None,
                respect_limits=respect_limits,
            )
        )

    skeleton.resetPositions()
    ik = ee.getOrCreateIK()
    dofs = list(range(dof))
    ik.setDofs(dofs)
    analytical = ik.setPythonAnalytical(solve, dofs, "ssik.prebuilt")
    limits = getattr(module, "_JOINT_LIMITS", [None] * dof)
    q_inside_limits = np.array(
        [0.0 if bound is None else 0.5 * sum(bound) for bound in limits]
    )
    # Some ssik home poses lie outside their joint limits; exercise both modes.
    targets = (
        (True, np.asarray(module.fk(q_inside_limits)).reshape((4, 4))),
        (False, np.asarray(module.T_HOME).reshape((4, 4))),
    )
    for respect_limits, target in targets:
        skeleton.setPositions(q)
        solutions = analytical.getSolutions(scene_module.isometry(target))
        assert solutions, specs[arm_index].module
        valid = [
            solution
            for solution in solutions
            if solution.mValidity == dart.dynamics.InverseKinematicsAnalytical.VALID
        ]
        skeleton.setPositions((valid or solutions)[0].mConfig)
        assert np.linalg.norm(ee.getWorldTransform().matrix() - target) < 1e-3


@requires_viewer
def test_ssik_panel_controls_dragging_and_solution_diagnostics(controls, monkeypatch):
    pytest.importorskip("ssik")
    from types import SimpleNamespace

    module = importlib.import_module("ssik.prebuilt.ur5_ik")
    calls = []

    def solve(target, **kwargs):
        calls.append((np.asarray(target).copy(), kwargs.copy()))
        count = min(20, kwargs.get("max_solutions") or 20)
        return [
            SimpleNamespace(
                q=np.full(module.DOF, 0.01 * i),
                fk_residual=1e-9 * i,
                refinement_used="refined" if kwargs["allow_refinement"] else "none",
            )
            for i in range(count)
        ]

    monkeypatch.setattr(module, "solve", solve)
    setup = _scene_module().build()
    assert not setup.key_actions
    assert not setup.allow_simulation
    context = _Context()
    setup.on_activate(context)
    assert len(context.attachments) == 1
    assert isinstance(context.attachments[0], dart.gui.GridVisual)
    assert context.attachments[0].getPlaneType() == dart.gui.GridVisual.PlaneType.XY
    assert len(context.drag) == 1
    target = context.drag[0]
    assert target.getName() == "ssik_target"
    world = setup.world
    assert world.getNumSkeletons() == 16
    assert world.getNumSimpleFrames() == 2
    arm_choice = _control(controls, "Arm")
    assert arm_choice.options == [arm.label for arm in get_ssik_arm_specs()]
    assert len(arm_choice.options) == 19
    assert calls[-1][1] == {
        "max_solutions": None,
        "respect_limits": True,
        "allow_refinement": False,
        "seed_metric": "wrap_linf",
    }
    browser = _control(controls, "Solution")
    assert browser.max == 19 and browser.visible
    _edit(browser, 3)
    np.testing.assert_array_equal(world.getSkeleton(3).getPositions(), np.full(6, 0.03))
    assert any(control.label == "fk residual: 3.000e-09" for control in controls)
    assert any(
        control.label == "q: " + ", ".join(["0.0300"] * 6) for control in controls
    )
    solved = world.getSimpleFrame("ssik_fk_solution")
    np.testing.assert_allclose(
        solved.getTransform().matrix(), module.fk(np.full(6, 0.03))
    )
    for index in (0, 3):
        rgba = (
            world.getSkeleton(index)
            .getBodyNode(0)
            .getShapeNode(0)
            .getVisualAspect()
            .getRGBA()
        )
        np.testing.assert_allclose(
            rgba,
            [0.90, 0.20, 0.18, 1.0] if index == 3 else [0.85, 0.32, 0.30, 0.22],
        )
    _edit(_control(controls, "Seed from selection"))
    assert _control(controls, "Use q seed").value
    np.testing.assert_array_equal(calls[-1][1]["q_seed"], np.full(6, 0.03))
    _edit(_control(controls, "Respect limits"), False)
    assert calls[-1][1]["respect_limits"] is False
    _edit(_control(controls, "Allow refinement"), True)
    assert calls[-1][1]["allow_refinement"] is True
    assert any(control.label == "refinement: refined" for control in controls)
    _edit(_control(controls, "Seed metric"), 1)
    assert calls[-1][1]["seed_metric"] == "wrap_l2"
    _edit(_control(controls, "Use q seed"), False)
    assert "q_seed" not in calls[-1][1]
    _edit(_control(controls, "Max solutions"), 2)
    assert calls[-1][1]["max_solutions"] == 2
    assert world.getNumSkeletons() == 2
    assert browser.value == 1 and browser.max == 1
    _edit(browser, -1)
    assert browser.value == 0
    _edit(browser, 1000)
    assert browser.value == 1
    _edit(_control(controls, "Max solutions"), 1)
    assert world.getNumSkeletons() == 1
    assert browser.value == 0 and not browser.visible
    assert _control(controls, "Solutions: 1")

    _edit(_control(controls, "Auto solve"), False)
    count = len(calls)
    position = _control(controls, "Position")
    rpy = _control(controls, "RPY deg")
    _edit(position, [0.4, 0.2, 0.3])
    _edit(rpy, [10, 20, 30])
    assert len(calls) == count
    np.testing.assert_array_equal(
        target.getTransform().translation(),
        np.asarray([0.4, 0.2, 0.3], dtype=np.float32),
    )
    np.testing.assert_allclose(
        target.getTransform().matrix(),
        make_ssik_target_transform(position.value, rpy.value).matrix(),
    )
    unchanged = target.getTransform().matrix().copy()
    _edit(position, [float("nan"), 0, 0])
    _edit(rpy, [0, float("inf"), 0])
    _edit(position, [1e100, 0, 0])
    _edit(rpy, [0, -1e100, 0])
    np.testing.assert_array_equal(
        position.value, np.asarray([0.4, 0.2, 0.3], dtype=np.float32)
    )
    np.testing.assert_array_equal(rpy.value, [10, 20, 30])
    np.testing.assert_array_equal(target.getTransform().matrix(), unchanged)
    _edit(_control(controls, "Solve"))
    assert len(calls) == count + 1
    np.testing.assert_array_equal(calls[-1][0], unchanged)
    _edit(_control(controls, "Home target"))
    home_position, home_rpy = ssik_target_controls(
        _scene_module().isometry(module.T_HOME)
    )
    np.testing.assert_array_equal(position.value, home_position)
    np.testing.assert_array_equal(rpy.value, home_rpy)
    dragged = dart.math.Isometry3()
    dragged.set_translation([0.31, -0.12, 0.53])
    target.setTransform(dragged)
    _edit(_control(controls, "Read dragged"))
    np.testing.assert_array_equal(
        position.value, np.asarray([0.31, -0.12, 0.53], dtype=np.float32)
    )
    assert len(calls) == count + 1
    setup.pre_refresh()
    _edit(_control(controls, "Auto solve"), True)
    assert len(calls) == count + 2
    dragged.set_translation([0.33, -0.12, 0.53])
    target.setTransform(dragged)
    setup.pre_refresh()
    assert len(calls) == count + 3
    np.testing.assert_array_equal(calls[-1][0], dragged.matrix())
    setup.pre_refresh()
    assert len(calls) == count + 3

    for arm_index, spec in enumerate(get_ssik_arm_specs()):
        _edit(arm_choice, arm_index)
        setup.pre_refresh()
        assert _control(controls, "Module: ssik.prebuilt." + spec.module).visible
        assert world.getNumSkeletons() > 0
        assert (
            world.getSkeleton(0).getNumDofs()
            == importlib.import_module("ssik.prebuilt." + spec.module).DOF
        )


@pytest.mark.parametrize("failing_method", ["solve", "fk"])
@requires_viewer
def test_ssik_solver_and_arm_load_failures_remain_panel_status(
    controls, monkeypatch, failing_method
):
    pytest.importorskip("ssik")
    scene_module = _scene_module()
    setup = scene_module.build()
    setup.on_activate(_Context())
    module = importlib.import_module("ssik.prebuilt.ur5_ik")

    def fail(*args, **kwargs):
        raise RuntimeError("solver unavailable")

    monkeypatch.setattr(module, failing_method, fail)
    _edit(_control(controls, "Solve"))
    assert _control(controls, "Solve failed: solver unavailable")
    assert not _control(controls, "Solution").visible
    assert setup.world.getNumSkeletons() == 1
    np.testing.assert_array_equal(
        setup.world.getSkeleton(0).getPositions(), np.zeros(6)
    )
    rgba = (
        setup.world.getSkeleton(0)
        .getBodyNode(0)
        .getShapeNode(0)
        .getVisualAspect()
        .getRGBA()
    )
    np.testing.assert_allclose(rgba, [0.55, 0.58, 0.62, 0.5])
    original_import = scene_module.importlib.import_module

    def failed_arm(name, *args, **kwargs):
        if name == "ssik.prebuilt.puma560_ik":
            raise ImportError("arm unavailable")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(scene_module.importlib, "import_module", failed_arm)
    _edit(_control(controls, "Arm"), 1)
    assert setup.world.getNumSkeletons() == 0
    assert _control(controls, "ssik is not available").visible
    assert _control(
        controls,
        "Install ssik in this Python environment to use this scene: arm unavailable",
    )
    np.testing.assert_array_equal(
        _control(controls, "Position").value,
        np.asarray([0.45, 0, 0.35], dtype=np.float32),
    )
    _edit(_control(controls, "Arm"), 0)
    assert not _control(controls, "ssik is not available").visible


@requires_viewer
def test_ssik_unavailable_factory_is_skipped(monkeypatch):
    scene_module = _scene_module()
    original_import = scene_module.importlib.import_module

    def missing(name, *args, **kwargs):
        if name == "ssik":
            raise ImportError("optional solver absent")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(scene_module.importlib, "import_module", missing)
    assert scene_module.scenes()[0].id == "ssik_ik_gui"
    with pytest.raises(viewer.AssetsNotPrefetchedError, match="Install ssik"):
        scene_module.build()


@requires_viewer
def test_ssik_solver_only_module_and_prismatic_chain_metadata(controls, monkeypatch):
    from types import SimpleNamespace

    scene_module = _scene_module()
    module = SimpleNamespace(
        BASE_LINK="base",
        EE_LINK="tip",
        DOF=1,
        T_HOME=np.eye(4),
        solve=lambda target, **kwargs: [SimpleNamespace(q=np.array([0.4]))],
    )
    monkeypatch.setattr(scene_module.importlib, "import_module", lambda name: module)
    setup = scene_module.build()
    setup.on_activate(_Context())
    assert setup.world.getNumSkeletons() == 0
    assert _control(controls, "Module: ssik.prebuilt.ur5_ik").visible
    assert _control(controls, "Solutions: 1 (all branches)")
    assert _control(controls, "q: 0.4000").visible
    assert not _control(controls, "ssik is not available").visible
    assert _control(controls, "fk residual: nan").visible
    assert not any(
        control.visible and control.label.startswith("FK xyz:") for control in controls
    )
    for joint_type, expected in (
        ("prismatic", True),
        ("PrismaticJoint", True),
        ("revolute", False),
    ):
        module._JOINT_T_LEFTS = [np.eye(4)]
        module._JOINT_T_RIGHTS = [np.eye(4)]
        module._JOINT_AXES = [[0, 0, 1]]
        module._JOINT_TYPES = [joint_type]
        assert scene_module.arm_chain(module)[0].prismatic is expected
    for name in ("_JOINT_T_LEFTS", "_JOINT_T_RIGHTS", "_JOINT_AXES", "_JOINT_TYPES"):
        value = getattr(module, name)
        delattr(module, name)
        assert scene_module.arm_chain(module) == []
        setattr(module, name, value)
