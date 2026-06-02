from __future__ import annotations

from types import SimpleNamespace

import dartpy as dart
import numpy as np
import pytest
from examples.demos._sx_bridge import SxRenderBridge
from examples.demos.runner import (
    DEFAULT_SCENE_BUILD_TIMEOUT_MS,
    PythonDemoScene,
    ScenePanel,
    SceneSetup,
    _make_world_factory,
    _scene_build_timeout_ms,
    _validate_scene,
)
from examples.demos.scenes._simbicon_robots import make_simbicon_panel

from examples.demos.scenes import (
    arm_push_box,
    atlas_simbicon,
    biped_stand,
    cartpole_gym_env,
    cartpole_mpc,
    diff_cartpole_trajopt,
    diff_drone_liftoff,
    diff_throw_to_target,
    experimental_rigid_body_gui,
    hybrid_dynamics,
    ipc_deformable_capsule_rod,
    ipc_deformable_cg_contact,
    ipc_deformable_cg_solver,
    ipc_deformable_drape,
    ipc_deformable_fcr_twist,
    ipc_deformable_fem_bar,
    ipc_deformable_fem_box,
    ipc_deformable_fem_buckle,
    ipc_deformable_fem_drop,
    ipc_deformable_fem_msh,
    ipc_deformable_fem_sphere,
    ipc_deformable_fem_twist,
    ipc_deformable_friction_slide,
    ipc_deformable_net,
    ipc_deformable_obj_cloth,
    ipc_deformable_plate_friction,
    ipc_deformable_pt_particles,
    ipc_deformable_rod_friction,
    ipc_deformable_scripted_dirichlet,
    ipc_deformable_seg_strand,
    ipc_deformable_trampoline,
    joint_constraints,
    legged_balance,
    operational_space_control,
    sensor_descriptors,
    sx_articulated,
    sx_contact,
    sx_floating_base,
    sx_rigid_fixed_joint,
    sx_rigid_ipc,
    sx_rigid_ipc_incline,
    sx_rigid_ipc_pile,
    sx_rigid_ipc_tunnel,
    sx_variational_chain,
    sx_variational_tumbler,
    vbd_beam,
    vbd_cloth,
    vbd_net,
    vbd_obstacle_drape,
    vbd_self_fold,
    vbd_tilted_strand,
    vehicle,
)


def _require_simulation_experimental_symbols(*names: str):
    try:
        import dartpy.simulation_experimental as sx
    except Exception as exc:  # pragma: no cover - reduced build without submodule
        pytest.skip(f"dartpy.simulation_experimental unavailable: {exc}")
    missing = [name for name in names if not hasattr(sx, name)]
    if missing:
        formatted = ", ".join(f"simulation_experimental.{name}" for name in missing)
        pytest.skip(f"{formatted} unavailable in this build")
    return sx


def test_make_world_factory_returns_panels_tuple() -> None:
    panel = ScenePanel("Controls", lambda builder, context: None)
    floating_panel = ScenePanel(
        "Floating", lambda builder, context: None, dock_side="none"
    )

    assert panel.dock_side == "right"
    assert panel.initial_size == (320.0, 440.0)
    assert panel.auto_resize is False
    assert floating_panel.dock_side == "none"

    scene = PythonDemoScene(
        id="panel_scene",
        title="Panel Scene",
        category="Tests",
        summary="Has a custom panel.",
        build=lambda: SceneSetup(world=object(), panels=[panel]),
    )

    result = _make_world_factory(scene)()

    assert isinstance(result, tuple)
    assert result == (result[0], None, None, [panel])


def test_validate_scene_accepts_hyphenated_scene_id_alias() -> None:
    scene = PythonDemoScene(
        id="sx_rigid_ipc_slide",
        title="Panel Scene",
        category="Tests",
        summary="Has a custom panel.",
        build=lambda: SceneSetup(world=object()),
    )

    _validate_scene("sx-rigid-ipc-slide", [scene])


def test_scene_build_timeout_follows_demo_startup_budget_by_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", raising=False)
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "37")

    assert _scene_build_timeout_ms() == 37.0


def test_scene_build_timeout_can_use_python_specific_override(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "37")
    monkeypatch.setenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", "11")

    assert _scene_build_timeout_ms() == 11.0


def test_scene_build_timeout_disable_requires_python_specific_override(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", raising=False)
    monkeypatch.setenv("DART_DEMO_SCENE_STARTUP_TIMEOUT_MS", "0")
    assert _scene_build_timeout_ms() == DEFAULT_SCENE_BUILD_TIMEOUT_MS

    monkeypatch.setenv("DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS", "0")
    assert _scene_build_timeout_ms() is None


class _FakeWorld:
    time_step = 0.001

    def __init__(self) -> None:
        self.steps = 0

    def step(self) -> None:
        self.steps += 1


class _FakeRigidBody:
    name = "dynamic"
    is_static = False

    def __init__(self) -> None:
        self.transform = np.eye(4)
        self.translation = np.zeros(3)
        self.force = np.zeros(3)
        self.torque = np.zeros(3)
        self.applied_forces: list[np.ndarray] = []
        self.applied_torques: list[np.ndarray] = []

    def apply_force(self, force: np.ndarray) -> None:
        value = np.asarray(force, dtype=float).reshape(3)
        self.applied_forces.append(value.copy())
        self.force = self.force + value

    def apply_torque(self, torque: np.ndarray) -> None:
        value = np.asarray(torque, dtype=float).reshape(3)
        self.applied_torques.append(value.copy())
        self.torque = self.torque + value

    def clear_force(self) -> None:
        self.force = np.zeros(3)

    def clear_torque(self) -> None:
        self.torque = np.zeros(3)


class _FakePanelBuilder:
    def __init__(self) -> None:
        self.events: list[str] = []

    def text(self, value: str) -> None:
        self.events.append(f"text:{value}")

    def slider(
        self, label: str, value: float, minimum: float, maximum: float
    ) -> tuple[bool, float]:
        self.events.append(f"slider:{label}:{minimum}:{maximum}")
        return False, value

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        self.events.append(f"checkbox:{label}")
        return False, value

    def button(self, label: str) -> bool:
        self.events.append(f"button:{label}")
        return False

    def plot_lines(self, label: str, values: list[float]) -> None:
        self.events.append(f"plot:{label}:{len(values)}")

    def separator(self) -> None:
        self.events.append("separator")


def test_high_value_sx_scenes_expose_custom_panels() -> None:
    _require_simulation_experimental_symbols("World")

    for scene_module, expected_title in (
        (sx_articulated, "Articulated sx"),
        (sx_floating_base, "Floating Base sx"),
        (sx_contact, "Contact sx"),
        (experimental_rigid_body_gui, "Rigid Bodies sx"),
        (sx_rigid_fixed_joint, "Rigid Fixed Joint"),
        (sx_rigid_ipc, "Rigid IPC Contact"),
        (sx_rigid_ipc_incline, "Rigid IPC Incline"),
        (sx_rigid_ipc_pile, "Rigid IPC Pile"),
        (sx_rigid_ipc_tunnel, "Rigid IPC Tunnel"),
        (sx_variational_chain, "Variational Chain"),
        (sx_variational_tumbler, "Variational Tumbler"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert setup.force_drag is not None
        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith("plot:") for event in builder.events)
        assert "text:External force" in builder.events
        assert any(event.startswith("text:drag target: ") for event in builder.events)
        assert "checkbox:Enable external force" in builder.events


def test_ipc_deformable_scene_exposes_diagnostics_panel() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableEdge", "World"
    )

    setup = ipc_deformable_friction_slide.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["IPC Friction Slide"]

    setup.panels[0].build(builder, object())

    assert any(event.startswith("plot:Min z:") for event in builder.events)
    assert any(
        event.startswith("text:solver: deformable IPC") for event in builder.events
    )


def test_diff_drone_scene_exposes_replay_panel() -> None:
    _require_simulation_experimental_symbols("World")

    setup = diff_drone_liftoff.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["Diff Drone Lift-Off"]

    setup.panels[0].build(builder, object())

    assert "slider:Playback stride:1.0:8.0" in builder.events
    assert "button:Reset replay" in builder.events
    assert any(event.startswith("plot:Aware height:") for event in builder.events)


def test_diff_trajectory_scenes_expose_replay_panels() -> None:
    _require_simulation_experimental_symbols("World")

    for scene_module, expected_title, expected_plot in (
        (diff_throw_to_target, "Diff Throw Target", "plot:Target distance:"),
        (diff_cartpole_trajopt, "Diff Cartpole TrajOpt", "plot:Cart x:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert "slider:Playback stride:1.0:8.0" in builder.events
        assert "button:Reset replay" in builder.events
        assert any(event.startswith(expected_plot) for event in builder.events)


def test_ipc_fem_buckle_scene_exposes_compression_panel() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableTetrahedron", "World"
    )

    setup = ipc_deformable_fem_buckle.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["IPC FEM Buckle"]

    setup.panels[0].build(builder, object())

    assert any(event.startswith("text:compression:") for event in builder.events)
    assert any(event.startswith("plot:Span x:") for event in builder.events)
    assert any(
        event.startswith("text:solver: deformable IPC") for event in builder.events
    )


def test_ipc_fem_sphere_scene_exposes_clearance_panel() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableTetrahedron", "World"
    )

    setup = ipc_deformable_fem_sphere.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["IPC FEM Sphere"]

    setup.panels[0].build(builder, object())

    assert any(event.startswith("text:sphere clearance:") for event in builder.events)
    assert any(event.startswith("plot:Sphere clearance:") for event in builder.events)
    assert any(
        event.startswith("text:solver: deformable IPC") for event in builder.events
    )


def test_ipc_friction_obstacle_scenes_expose_speed_panels() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableEdge", "World"
    )

    for scene_module, expected_title, expected_plot in (
        (ipc_deformable_plate_friction, "IPC Plate Friction", "plot:X speed:"),
        (ipc_deformable_rod_friction, "IPC Rod Friction", "plot:Rod-axis speed:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith(expected_plot) for event in builder.events)
        assert any(
            event.startswith("text:friction coefficient:") for event in builder.events
        )
        assert any(
            event.startswith("text:solver: deformable IPC") for event in builder.events
        )


def test_ipc_drape_showcase_scenes_expose_shape_panels() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableEdge", "World"
    )

    for scene_module, expected_title, expected_plot in (
        (ipc_deformable_capsule_rod, "IPC Capsule Rod", "plot:Rod clearance:"),
        (ipc_deformable_trampoline, "IPC Trampoline", "plot:Center height:"),
        (ipc_deformable_drape, "IPC Drape", "plot:Ground clearance:"),
        (ipc_deformable_net, "IPC Net", "plot:Net sag:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith(expected_plot) for event in builder.events)
        assert any(
            event.startswith("text:solver: deformable IPC") for event in builder.events
        )


def test_ipc_cg_showcase_scenes_expose_solver_panels() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableTetrahedron", "World"
    )

    for scene_module, expected_title, expected_plot in (
        (ipc_deformable_cg_solver, "IPC FEM CG Solver", "plot:Tip drop:"),
        (ipc_deformable_cg_contact, "IPC FEM CG Contact", "plot:Ground clearance:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert "text:solver: deformable IPC" in builder.events
        assert any(event.startswith("text:linear solve:") for event in builder.events)
        assert any(event.startswith(expected_plot) for event in builder.events)


def test_vbd_showcase_scenes_expose_solver_panels() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions", "DeformableSolverOptions", "World"
    )

    for scene_module, expected_title, expected_plot in (
        (vbd_cloth, "VBD Cloth", "plot:Cloth sag:"),
        (vbd_net, "VBD Net", "plot:Net sag:"),
        (vbd_beam, "VBD Beam", "plot:Tip sag:"),
        (vbd_tilted_strand, "VBD Tilted Strand", "plot:Free-end drop:"),
        (vbd_obstacle_drape, "VBD Sphere Drape", "plot:Sphere clearance:"),
        (vbd_self_fold, "VBD Self Contact", "plot:Layer clearance:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith(expected_plot) for event in builder.events)
        assert any(event.startswith("text:solver iters:") for event in builder.events)


def test_ipc_asset_and_scripted_scenes_expose_diagnostics_panels() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions",
        "DeformableDirichletBoundaryCondition",
        "DeformableEdge",
        "World",
        "load_obj_triangle_mesh",
        "load_point_set",
        "load_seg_line_mesh",
    )

    for scene_module, expected_title, expected_plot in (
        (ipc_deformable_obj_cloth, "IPC OBJ Cloth", "plot:Cloth sag:"),
        (ipc_deformable_seg_strand, "IPC SEG Strand", "plot:Tip drop:"),
        (ipc_deformable_pt_particles, "IPC PT Particles", "plot:Ground clearance:"),
        (
            ipc_deformable_scripted_dirichlet,
            "IPC Scripted Banner",
            "plot:Out-of-plane span:",
        ),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith(expected_plot) for event in builder.events)
        assert any(
            event.startswith("text:solver: deformable IPC") for event in builder.events
        )


def test_ipc_fem_scenes_expose_diagnostics_panels() -> None:
    _require_simulation_experimental_symbols(
        "DeformableBodyOptions",
        "DeformableTetrahedron",
        "World",
        "load_gmsh_tet_mesh",
    )

    for scene_module, expected_title, expected_plot in (
        (ipc_deformable_fem_bar, "IPC FEM Bar", "plot:Tip drop:"),
        (ipc_deformable_fem_twist, "IPC FEM Twist", "plot:Span y:"),
        (ipc_deformable_fcr_twist, "IPC FCR Twist", "plot:Span y:"),
        (ipc_deformable_fem_drop, "IPC FEM Drop", "plot:Ground clearance:"),
        (ipc_deformable_fem_box, "IPC FEM Box", "plot:Box clearance:"),
        (ipc_deformable_fem_msh, "IPC FEM MSH", "plot:Tip drop:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith(expected_plot) for event in builder.events)
        assert any(
            event.startswith("text:solver: deformable IPC") for event in builder.events
        )


def test_control_modern_scenes_expose_interactive_panels() -> None:
    for scene_module, expected_title, expected_plot in (
        (legged_balance, "Legged Balance", "plot:Angle:"),
        (arm_push_box, "Arm Push Box", "plot:Box x:"),
        (cartpole_gym_env, "Cart-pole Env", "plot:Cart x:"),
        (cartpole_mpc, "Cart-pole MPC", "plot:Force:"),
        (sensor_descriptors, "Sensor Descriptors", "text:status: descriptor surface"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith(expected_plot) for event in builder.events)
        if scene_module is not sensor_descriptors:
            assert setup.pre_step is not None
            assert setup.step is None


def test_legacy_control_scenes_expose_controller_panels() -> None:
    for scene_module, expected_title, expected_plot in (
        (
            operational_space_control,
            "Operational Space",
            "plot:Tracking error:",
        ),
        (hybrid_dynamics, "Hybrid Dynamics", "plot:Arm command:"),
        (biped_stand, "Biped Stand", "plot:Pose error:"),
        (joint_constraints, "Joint Constraints", "plot:Sagittal offset:"),
        (vehicle, "Vehicle", "plot:Steering angle:"),
        (atlas_simbicon, "Atlas SIMBICON", "plot:atlas z:"),
    ):
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert setup.pre_step is not None
        assert any(event.startswith(expected_plot) for event in builder.events)


class _FakeSimbiconController:
    def __init__(
        self,
        name: str,
        state: int,
        swing: str,
        pelvis_height: float,
        balance_sagittal: float,
        balance_coronal: float,
    ) -> None:
        self.cfg = SimpleNamespace(name=name)
        self._diagnostics = {
            "name": name,
            "state": state,
            "swing": swing,
            "control_enabled": True,
            "state_time": 0.125,
            "pelvis_height": pelvis_height,
            "balance_sagittal": balance_sagittal,
            "balance_sagittal_velocity": -0.02,
            "balance_coronal": balance_coronal,
            "balance_coronal_velocity": 0.03,
        }

    def diagnostics(self) -> dict[str, float | int | bool | str]:
        return self._diagnostics


def test_simbicon_panel_reports_duo_robot_diagnostics_without_assets() -> None:
    panel = make_simbicon_panel(
        "SIMBICON Duo",
        [
            _FakeSimbiconController("atlas", 1, "left", 0.91, -0.12, 0.03),
            _FakeSimbiconController("g1", 2, "right", 0.42, 0.04, -0.02),
        ],
    )
    builder = _FakePanelBuilder()

    panel.build(builder, object())

    assert "text:atlas: state 1 swing left" in builder.events
    assert "text:g1: state 2 swing right" in builder.events
    assert any(event.startswith("plot:atlas z:") for event in builder.events)
    assert any(event.startswith("plot:g1 z:") for event in builder.events)
    assert any(event.startswith("plot:g1 sag:") for event in builder.events)
    assert any(event.startswith("plot:g1 cor:") for event in builder.events)


@pytest.mark.skipif(
    not hasattr(dart, "gui") or not hasattr(dart.gui, "extract_renderables"),
    reason="GUI descriptor extraction is not available in this build",
)
def test_sx_bridge_force_drag_uses_renderable_id_and_restores_rigid_force() -> None:
    sx_world = _FakeWorld()
    target = _FakeRigidBody()
    other = _FakeRigidBody()
    other.name = "other"

    bridge = SxRenderBridge(sx_world, name="force_drag_test")
    bridge.add_rigid_body_visual(
        other,
        dart.BoxShape(np.array([0.1, 0.1, 0.1])),
        (0.5, 0.5, 0.5),
        name="repeated_visual",
    )
    bridge.add_rigid_body_visual(
        target,
        dart.BoxShape(np.array([0.1, 0.1, 0.1])),
        (0.2, 0.5, 0.8),
        name="repeated_visual",
    )
    target_frame_name = next(
        name for name, sx_object in bridge._by_name.items() if sx_object is target
    )
    target_id = next(
        int(renderable.id)
        for renderable in dart.gui.extract_renderables(bridge.render_world)
        if renderable.shape_frame_name == target_frame_name
    )

    bridge.force_drag(
        {
            "active": True,
            "renderable_id": target_id,
            "renderable_name": "repeated_visual",
            "force": np.array([2.0, 0.0, 0.0]),
            "application_point": np.array([0.0, 1.0, 0.0]),
        }
    )
    bridge.pre_step()

    assert sx_world.steps == 1
    assert bridge._last_drag_status == "applying"
    assert len(target.applied_forces) == 1
    assert np.allclose(target.applied_forces[0], [2.0, 0.0, 0.0])
    assert np.allclose(target.applied_torques[0], [0.0, 0.0, -2.0])
    assert np.allclose(target.force, [0.0, 0.0, 0.0])
    assert np.allclose(target.torque, [0.0, 0.0, 0.0])
    assert other.applied_forces == []

    bridge.force_drag(
        {
            "active": True,
            "renderable_id": target_id,
            "renderable_name": "repeated_visual",
            "force": np.array([np.nan, 0.0, 0.0]),
            "application_point": np.array([0.0, 1.0, 0.0]),
        }
    )
    bridge.pre_step()

    assert sx_world.steps == 2
    assert bridge._last_drag_status == "invalid event"
    assert bridge._last_drag_magnitude == 0.0
    assert len(target.applied_forces) == 1


@pytest.mark.skipif(
    not hasattr(dart, "gui") or not hasattr(dart.gui, "extract_renderables"),
    reason="GUI descriptor extraction is not available in this build",
)
def test_sx_bridge_external_force_panel_reports_disabled_and_static_targets() -> None:
    sx_world = _FakeWorld()
    static_target = _FakeRigidBody()
    static_target.name = "ground"
    static_target.is_static = True

    bridge = SxRenderBridge(sx_world, name="external_force_state_test")
    bridge.add_rigid_body_visual(
        static_target,
        dart.BoxShape(np.array([0.1, 0.1, 0.1])),
        (0.5, 0.5, 0.5),
        name="static_visual",
    )
    static_id = next(
        int(renderable.id)
        for renderable in dart.gui.extract_renderables(bridge.render_world)
        if renderable.shape_frame_name == "static_visual"
    )

    bridge.force_drag_enabled = False
    bridge.force_drag(
        {
            "active": True,
            "renderable_id": static_id,
            "renderable_name": "static_visual",
            "force": np.array([2.0, 0.0, 0.0]),
            "application_point": np.array([0.0, 1.0, 0.0]),
        }
    )
    disabled_builder = _FakePanelBuilder()
    bridge.build_control_panel(disabled_builder, object())

    assert "text:status: disabled" in disabled_builder.events
    assert "text:drag target: disabled" in disabled_builder.events
    assert "text:target: disabled" in disabled_builder.events
    assert static_target.applied_forces == []

    bridge.force_drag_enabled = True
    bridge.force_drag(
        {
            "active": True,
            "renderable_id": static_id,
            "renderable_name": "static_visual",
            "force": np.array([2.0, 0.0, 0.0]),
            "application_point": np.array([0.0, 1.0, 0.0]),
        }
    )
    static_builder = _FakePanelBuilder()
    bridge.build_control_panel(static_builder, object())

    assert "text:status: static target" in static_builder.events
    assert "text:drag target: none" in static_builder.events
    assert "text:target: ground" in static_builder.events
    assert "text:magnitude: 0.00 N" in static_builder.events
    assert static_target.applied_forces == []

    dynamic_target = _FakeRigidBody()
    dynamic_target.name = "box"
    dynamic_bridge = SxRenderBridge(_FakeWorld(), name="external_force_hint_test")
    dynamic_bridge.add_rigid_body_visual(
        static_target,
        dart.BoxShape(np.array([0.1, 0.1, 0.1])),
        (0.5, 0.5, 0.5),
        name="static_visual",
    )
    dynamic_bridge.add_rigid_body_visual(
        dynamic_target,
        dart.BoxShape(np.array([0.1, 0.1, 0.1])),
        (0.2, 0.5, 0.8),
        name="dynamic_visual",
    )
    hint_builder = _FakePanelBuilder()
    dynamic_bridge.build_control_panel(hint_builder, object())

    assert "text:drag target: box" in hint_builder.events
