from __future__ import annotations

import dartpy as dart
import numpy as np
import pytest
from examples.demos._world_bridge import WorldRenderBridge
from examples.demos.registry import make_demo_scenes
from examples.demos.runner import (
    CAPTURE_METRICS_INFO_KEY,
    DEFAULT_INITIAL_SCENE_ID,
    DEFAULT_SCENE_BUILD_TIMEOUT_MS,
    PythonDemoScene,
    REPLAY_TIMELINE_INFO_KEY,
    RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS,
    RIGID_VISUAL_WORKFLOW_GUIDES,
    ScenePanel,
    SceneSetup,
    _attach_replay_controls,
    _capture_metadata_mapping,
    _default_initial_scene_args,
    _has_world_replay_api,
    _make_world_factory,
    _rigid_workflow_packet_command,
    _rigid_workflow_row_packet_command,
    _rigid_workflow_row_video_packet_command,
    _rigid_workflow_viewer_command,
    _scene_build_timeout_ms,
    _workflow_matching_guides,
    _validate_scene,
)

from examples.demos.scenes import (
    articulated,
    atlas_simbicon,
    contact,
    diff_cartpole_trajopt,
    diff_drone_liftoff,
    diff_pre_contact_surrogate,
    diff_throw_to_target,
    floating_base,
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
    plan083_unified_newton_barrier,
    planned,
    replay_scrubber,
    rigid_body,
    rigid_body_modes,
    rigid_collision_casts,
    rigid_collision_query_options,
    rigid_contact_inspector,
    rigid_contact_manipulation,
    rigid_contact_scale_budget,
    rigid_contact_solver_compare,
    rigid_distance_spring,
    rigid_executor_equivalence,
    rigid_external_loads,
    rigid_fixed_joint,
    rigid_frame_hierarchy,
    rigid_friction_threshold,
    rigid_free_flight,
    rigid_ipc,
    rigid_ipc_edge_drop,
    rigid_ipc_incline,
    rigid_ipc_slide,
    rigid_ipc_stack_packet,
    rigid_joint_breakage,
    rigid_joint_motor_limits,
    rigid_joint_passive_parameters,
    rigid_kinematic_driver,
    rigid_kinematic_normal_push,
    rigid_link_center_of_mass,
    rigid_link_jacobian,
    rigid_link_point_loads,
    rigid_limited_joints,
    rigid_loop_closure,
    rigid_material_mixing,
    rigid_multibody_dynamics_terms,
    rigid_multibody_solver_family,
    rigid_restitution_ladder,
    rigid_screw_joint_pitch,
    rigid_ipc_pile,
    rigid_ipc_tunnel,
    rigid_solver_compare,
    rigid_spin_roll_coupling,
    rigid_stack_stability,
    rigid_step_diagnostics,
    rigid_timestep_sensitivity,
    robot_puppets,
    variational_chain,
    variational_tumbler,
    vbd_beam,
    vbd_cloth,
    vbd_net,
    vbd_obstacle_drape,
    vbd_self_fold,
    vbd_tilted_strand,
)


def _require_simulation_symbols(*names: str):
    try:
        import dartpy as sx
    except Exception as exc:  # pragma: no cover - reduced build without submodule
        pytest.skip(f"dartpy unavailable: {exc}")
    missing = [name for name in names if not hasattr(sx, name)]
    if missing:
        formatted = ", ".join(f"dartpy.{name}" for name in missing)
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
    assert result == (None, None, [panel], None)


def test_make_world_factory_injects_shared_replay_panel_for_world_scenes() -> None:
    replay_world = _FakeReplayWorld()
    bridge = _FakeReplayBridge(replay_world)
    captured_setup: dict[str, SceneSetup] = {}

    def build() -> SceneSetup:
        setup = SceneSetup(
            world=bridge.render_world,
            pre_step=bridge.pre_step,
            panels=[],
            info={"sx_world": replay_world},
        )
        captured_setup["value"] = setup
        return setup

    scene = PythonDemoScene(
        id="replayable",
        title="Replayable",
        category="Tests",
        summary="Uses an DART 7 World.",
        build=build,
    )

    result = _make_world_factory(scene)()

    assert isinstance(result, tuple)
    pre_step, force_drag, panels, renderable_provider = result
    assert renderable_provider is None
    assert force_drag is None
    assert [panel.title for panel in panels] == ["Replay"]
    assert replay_world.replay_recording_enabled is True
    assert replay_world.replay_frame_count == 1
    assert bridge.sync_calls == 1

    pre_step()

    assert replay_world.steps == 1
    assert replay_world.replay_frame_count == 2
    assert bridge.sync_calls == 2
    assert captured_setup["value"].info["replay_panel_title"] == "Replay"


def test_capture_metadata_projects_replay_timeline_for_manifests() -> None:
    replay_world = _FakeReplayWorld()
    bridge = _FakeReplayBridge(replay_world)
    setup = SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        info={
            "sx_world": replay_world,
            REPLAY_TIMELINE_INFO_KEY: {
                "signal_label": "Diagnostic gap",
                "signal": lambda snapshot: 1.0,
                "markers": lambda snapshot: 0.0,
            },
        },
    )
    scene = PythonDemoScene(
        id="replayable",
        title="Replayable",
        category="Tests",
        summary="Uses a scene-owned Replay timeline.",
        build=lambda: setup,
    )

    setup = _attach_replay_controls(scene, setup)
    metadata = _capture_metadata_mapping(setup)

    assert metadata == {
        "replay_timeline": {
            "has_markers": True,
            "has_signal": True,
            "panel": "Replay",
            "signal_label": "Diagnostic gap",
        }
    }


def test_shared_replay_panel_scrubs_and_replays_saved_world_states() -> None:
    replay_world = _FakeReplayWorld()
    sync_calls = {"count": 0}

    def sync() -> None:
        sync_calls["count"] += 1

    def live_pre_step() -> None:
        replay_world.step()

    scene = PythonDemoScene(
        id="replayable",
        title="Replayable",
        category="Tests",
        summary="Uses an DART 7 World.",
        build=lambda: SceneSetup(
            world=object(),
            pre_step=live_pre_step,
            info={
                "sx_world": replay_world,
                "replay_sync": sync,
                "replay_live_step_is_stateless": True,
            },
        ),
    )
    setup = _attach_replay_controls(scene, scene.build())
    panel = setup.panels[-1]
    context = _FakePanelContext()

    setup.pre_step()
    setup.pre_step()
    assert replay_world.replay_frame_count == 3
    assert replay_world.frame == 2

    scrub_builder = _ScriptedPanelBuilder(
        timeline_values={"Saved states##py_demo_replay_timeline": 1.0}
    )
    panel.build(scrub_builder, context)

    assert replay_world.replay_cursor == 1
    assert replay_world.frame == 1
    assert context.paused is True
    assert any(
        event.startswith("timeline:Saved states##py_demo_replay_timeline")
        for event in scrub_builder.events
    )

    play_builder = _ScriptedPanelBuilder(clicked_buttons={"Play##replay_play"})
    panel.build(play_builder, context)
    assert context.paused is False

    setup.pre_step()

    assert replay_world.replay_cursor == 2
    assert replay_world.frame == 2
    assert replay_world.steps == 2

    resume_builder = _ScriptedPanelBuilder(clicked_buttons={"Resume live"})
    panel.build(resume_builder, context)
    setup.pre_step()

    assert replay_world.steps == 3
    assert replay_world.frame == 3
    assert sync_calls["count"] >= 3


def test_shared_replay_panel_uses_default_timeline_without_metadata() -> None:
    replay_world = _FakeReplayWorld()

    def live_pre_step() -> None:
        replay_world.step()

    scene = PythonDemoScene(
        id="default_timeline",
        title="Default Timeline",
        category="Tests",
        summary="Uses the generic replay timeline.",
        build=lambda: SceneSetup(
            world=object(),
            pre_step=live_pre_step,
            info={
                "sx_world": replay_world,
                "replay_live_step_is_stateless": True,
            },
        ),
    )
    setup = _attach_replay_controls(scene, scene.build())
    setup.pre_step()
    setup.pre_step()

    builder = _FakePanelBuilder()
    setup.panels[-1].build(builder, _FakePanelContext())

    timeline = builder.timelines[-1]
    assert timeline["label"] == "Saved states##py_demo_replay_timeline"
    assert timeline["value_track"] == []
    assert len(timeline["marker_track"]) == replay_world.replay_frame_count
    assert any(float(value) > 0.0 for value in timeline["marker_track"])
    assert len(timeline["cursor_track"]) == replay_world.replay_frame_count
    assert timeline["value_track_label"] == "Saved states"


def test_shared_replay_panel_uses_scene_replay_timeline_metadata() -> None:
    replay_world = _FakeReplayWorld()
    controller_state = {"signal": 0.0}

    def live_pre_step() -> None:
        controller_state["signal"] += 1.0
        replay_world.step()

    def capture_state() -> dict[str, float]:
        return dict(controller_state)

    def restore_state(snapshot: dict[str, float]) -> None:
        controller_state["signal"] = float(snapshot["signal"])

    scene = PythonDemoScene(
        id="diagnostic_timeline",
        title="Diagnostic Timeline",
        category="Tests",
        summary="Provides a guided replay diagnostic.",
        build=lambda: SceneSetup(
            world=object(),
            pre_step=live_pre_step,
            info={
                "sx_world": replay_world,
                "replay_capture_state": capture_state,
                "replay_restore_state": restore_state,
                "replay_timeline": {
                    "signal_label": "Synthetic signal",
                    "signal": lambda snapshot: snapshot["signal"],
                    "markers": lambda snapshot: snapshot["signal"] >= 2.0,
                },
            },
        ),
    )
    setup = _attach_replay_controls(scene, scene.build())
    for _ in range(3):
        setup.pre_step()

    builder = _FakePanelBuilder()
    setup.panels[-1].build(builder, _FakePanelContext())

    timeline = builder.timelines[-1]
    assert timeline["label"] == "Saved states##py_demo_replay_timeline"
    assert timeline["value_track"] == [0.0, 1.0, 2.0, 3.0]
    assert timeline["marker_track"] == [0.0, 0.0, 1.0, 1.0]
    assert timeline["cursor_track"] == [0.0, 0.0, 0.0, 1.0]
    assert timeline["value_track_label"] == "Synthetic signal"


def test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata() -> None:
    replay_world = _FakeReplayWorld()
    controller_state = {"signal": 0.0}

    def live_pre_step() -> None:
        controller_state["signal"] += 1.0
        replay_world.step()

    def capture_state() -> dict[str, float]:
        return dict(controller_state)

    def restore_state(snapshot: dict[str, float]) -> None:
        controller_state["signal"] = float(snapshot["signal"])

    scene = PythonDemoScene(
        id="signal_only_timeline",
        title="Signal Only Timeline",
        category="Tests",
        summary="Provides a signal without custom markers.",
        build=lambda: SceneSetup(
            world=object(),
            pre_step=live_pre_step,
            info={
                "sx_world": replay_world,
                "replay_capture_state": capture_state,
                "replay_restore_state": restore_state,
                "replay_timeline": {
                    "signal_label": "Signal only",
                    "signal": lambda snapshot: snapshot["signal"],
                },
            },
        ),
    )
    setup = _attach_replay_controls(scene, scene.build())
    setup.pre_step()
    setup.pre_step()

    builder = _FakePanelBuilder()
    setup.panels[-1].build(builder, _FakePanelContext())

    timeline = builder.timelines[-1]
    assert timeline["value_track"] == [0.0, 1.0, 2.0]
    assert len(timeline["marker_track"]) == replay_world.replay_frame_count
    assert any(float(value) > 0.0 for value in timeline["marker_track"])
    assert timeline["value_track_label"] == "Signal only"


def test_shared_replay_panel_ignores_malformed_timeline_metadata() -> None:
    replay_world = _FakeReplayWorld()

    def live_pre_step() -> None:
        replay_world.step()

    def capture_state() -> dict[str, float]:
        return {}

    def restore_state(_snapshot: dict[str, float]) -> None:
        pass

    scene = PythonDemoScene(
        id="malformed_timeline",
        title="Malformed Timeline",
        category="Tests",
        summary="Falls back when timeline metadata is invalid.",
        build=lambda: SceneSetup(
            world=object(),
            pre_step=live_pre_step,
            info={
                "sx_world": replay_world,
                "replay_capture_state": capture_state,
                "replay_restore_state": restore_state,
                "replay_timeline": {
                    "signal_label": "Broken signal",
                    "signal": lambda snapshot: snapshot["missing"],
                },
            },
        ),
    )
    setup = _attach_replay_controls(scene, scene.build())
    setup.pre_step()
    setup.pre_step()

    builder = _FakePanelBuilder()
    setup.panels[-1].build(builder, _FakePanelContext())

    timeline = builder.timelines[-1]
    assert timeline["value_track"] == []
    assert len(timeline["marker_track"]) == replay_world.replay_frame_count
    assert any(float(value) > 0.0 for value in timeline["marker_track"])
    assert timeline["value_track_label"] == "Saved states"


def test_shared_replay_panel_restores_scene_replay_state() -> None:
    replay_world = _FakeReplayWorld()
    controller_state = {"phase": 0}

    def live_pre_step() -> None:
        controller_state["phase"] += 1
        replay_world.step()

    def capture_state() -> dict[str, int]:
        return dict(controller_state)

    def restore_state(snapshot: dict[str, int]) -> None:
        controller_state["phase"] = int(snapshot["phase"])

    scene = PythonDemoScene(
        id="stateful_replay",
        title="Stateful Replay",
        category="Tests",
        summary="Keeps Python controller state outside the World.",
        build=lambda: SceneSetup(
            world=object(),
            pre_step=live_pre_step,
            info={
                "sx_world": replay_world,
                "replay_capture_state": capture_state,
                "replay_restore_state": restore_state,
            },
        ),
    )
    setup = _attach_replay_controls(scene, scene.build())
    panel = setup.panels[-1]
    context = _FakePanelContext()

    setup.pre_step()
    setup.pre_step()
    assert replay_world.replay_frame_count == 3
    assert controller_state["phase"] == 2

    panel.build(
        _ScriptedPanelBuilder(
            timeline_values={"Saved states##py_demo_replay_timeline": 1.0}
        ),
        context,
    )
    assert replay_world.frame == 1
    assert controller_state["phase"] == 1

    panel.build(_ScriptedPanelBuilder(clicked_buttons={"Resume live"}), context)
    setup.pre_step()

    assert replay_world.frame == 2
    assert controller_state["phase"] == 2


def test_custom_replay_pre_step_requires_state_hooks_or_stateless_flag() -> None:
    replay_world = _FakeReplayWorld()

    def live_pre_step() -> None:
        replay_world.step()

    setup = SceneSetup(
        world=object(),
        pre_step=live_pre_step,
        info={"sx_world": replay_world},
    )
    scene = PythonDemoScene(
        id="stateful_without_hooks",
        title="Stateful Without Hooks",
        category="Tests",
        summary="Has a custom live pre-step without replay state hooks.",
        build=lambda: setup,
    )

    _attach_replay_controls(scene, setup)

    assert setup.panels == []
    assert "replay_controller" not in setup.info
    assert (
        "custom pre_step needs replay_capture_state"
        in setup.info["shared_replay_skipped_reason"]
    )


def test_shared_replay_panel_honors_scene_opt_out() -> None:
    replay_world = _FakeReplayWorld()
    setup = SceneSetup(
        world=object(),
        info={"sx_world": replay_world, "disable_shared_replay": True},
    )
    scene = PythonDemoScene(
        id="custom_replay",
        title="Custom Replay",
        category="Tests",
        summary="Already owns replay UI.",
        build=lambda: setup,
    )

    assert _attach_replay_controls(scene, setup) is setup
    assert setup.panels == []
    assert "replay_controller" not in setup.info


def test_shared_replay_panel_accepts_physics_world_info_key() -> None:
    replay_world = _FakeReplayWorld()
    setup = SceneSetup(world=object(), info={"physics_world": replay_world})
    scene = PythonDemoScene(
        id="physics_world_scene",
        title="Physics World Scene",
        category="Tests",
        summary="Uses the alternate replay world key.",
        build=lambda: setup,
    )

    _attach_replay_controls(scene, setup)

    assert [panel.title for panel in setup.panels] == ["Replay"]
    assert setup.info["replay_world"] is replay_world


def test_registered_world_scenes_receive_shared_replay_controls() -> None:
    _require_simulation_symbols("World")
    render_only_scene_ids = {
        "planned_inverse_kinematics",
        "g1_puppet",
        "planned_simbicon_walking",
        "planned_operational_space_control",
        "planned_mobile_manipulation",
        "plan083_abd_complex_geometry",
        "plan083_abd_fem_coupling",
        "diff_throw_to_target",
        "diff_cartpole_trajopt",
        "diff_drone_liftoff",
        "diff_pre_contact_surrogate",
    }
    replay_attached: set[str] = set()
    replay_opt_out: set[str] = set()
    render_only_seen: set[str] = set()

    for scene in make_demo_scenes():
        setup = scene.build()
        has_replay_world = any(
            _has_world_replay_api(setup.info.get(key))
            for key in ("sx_world", "physics_world")
        ) or _has_world_replay_api(setup.world)

        setup = _attach_replay_controls(scene, setup)
        attached = "replay_controller" in setup.info

        if has_replay_world and setup.info.get("disable_shared_replay"):
            assert scene.id == "replay_scrubber"
            assert not attached
            replay_opt_out.add(scene.id)
        elif has_replay_world:
            assert attached, scene.id
            assert setup.info["replay_panel_title"] == "Replay"
            assert any(panel.title == "Replay" for panel in setup.panels)
            assert setup.info["replay_controller"].frame_count >= 1
            replay_attached.add(scene.id)
        else:
            assert not attached, scene.id
            render_only_seen.add(scene.id)

    assert replay_attached
    assert replay_opt_out == {"replay_scrubber"}
    assert render_only_seen == render_only_scene_ids


def test_validate_scene_accepts_hyphenated_scene_id_alias() -> None:
    scene = PythonDemoScene(
        id="rigid_ipc_slide",
        title="Panel Scene",
        category="Tests",
        summary="Has a custom panel.",
        build=lambda: SceneSetup(world=object()),
    )

    _validate_scene("rigid-ipc-slide", [scene])


def test_default_py_demos_launch_uses_rigid_body_front_door() -> None:
    assert DEFAULT_INITIAL_SCENE_ID == "rigid_body"
    assert _default_initial_scene_args([], None, {}) == [
        "--scene",
        "rigid_body",
    ]

    assert (
        _default_initial_scene_args(["--scene", "hello_world"], "hello_world", {}) == []
    )
    assert _default_initial_scene_args([], None, {"DART_DEMOS_SCENE": "boxes"}) == []
    assert _default_initial_scene_args(["--cycle-scenes"], None, {}) == []


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


class _FakeReplayWorld(_FakeWorld):
    def __init__(self) -> None:
        super().__init__()
        self.time = 0.0
        self.frame = 0
        self._recording_enabled = False
        self._frames: list[tuple[float, int]] = []
        self._cursor: int | None = None

    @property
    def replay_recording_enabled(self) -> bool:
        return self._recording_enabled

    @replay_recording_enabled.setter
    def replay_recording_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._recording_enabled:
            return
        self._recording_enabled = enabled
        if enabled:
            self._frames.clear()
            self._cursor = None
            self._record_frame()

    @property
    def replay_frame_count(self) -> int:
        return len(self._frames)

    @property
    def replay_cursor(self) -> int | None:
        return self._cursor

    def _record_frame(self) -> None:
        if self._cursor is not None and self._cursor + 1 < len(self._frames):
            del self._frames[self._cursor + 1 :]
        self._frames.append((self.time, self.frame))
        self._cursor = len(self._frames) - 1

    def step(self) -> None:
        super().step()
        self.frame += 1
        self.time += self.time_step
        if self._recording_enabled:
            self._record_frame()

    def restore_replay_frame(self, index: int) -> None:
        self._cursor = max(0, min(int(index), len(self._frames) - 1))
        self.time, self.frame = self._frames[self._cursor]

    def clear_replay_recording(self) -> None:
        self._frames.clear()
        self._cursor = None
        if self._recording_enabled:
            self._record_frame()

    def get_replay_frame_time(self, index: int) -> float:
        return self._frames[index][0]

    def get_replay_simulation_frame(self, index: int) -> int:
        return self._frames[index][1]


class _FakeReplayBridge:
    def __init__(self, world: _FakeReplayWorld) -> None:
        self._physics_world = world
        self.render_world = object()
        self.sync_calls = 0

    def sync(self) -> None:
        self.sync_calls += 1

    def pre_step(self) -> None:
        self._physics_world.step()
        self.sync()


class _FakePanelContext:
    def __init__(self) -> None:
        self.paused = False
        self.single_step_requests = 0
        self.scene_switch_requests: list[str] = []
        self.scene_replay_requests: list[str] = []

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)

    def request_single_step(self) -> None:
        self.single_step_requests += 1

    def request_scene_switch(self, scene_id: str) -> None:
        self.scene_switch_requests.append(scene_id)

    def request_scene_replay(self, scene_id: str) -> None:
        self.scene_replay_requests.append(scene_id)


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
        self.timelines: list[dict[str, object]] = []

    def text(self, value: str) -> None:
        self.events.append(f"text:{value}")

    def slider(
        self, label: str, value: float, minimum: float, maximum: float
    ) -> tuple[bool, float]:
        self.events.append(f"slider:{label}:{minimum}:{maximum}")
        return False, value

    def timeline(
        self,
        label: str,
        value: float,
        minimum: float,
        maximum: float,
        value_track: list[float] | tuple[float, ...] = (),
        marker_track: list[float] | tuple[float, ...] = (),
        cursor_track: list[float] | tuple[float, ...] = (),
        value_track_label: str = "Values",
    ) -> tuple[bool, float]:
        self.timelines.append(
            {
                "cursor_track": list(cursor_track),
                "label": label,
                "marker_track": list(marker_track),
                "value_track": list(value_track),
                "value_track_label": value_track_label,
            }
        )
        self.events.append(
            f"timeline:{label}:{minimum}:{maximum}:"
            f"{len(value_track)}:{len(marker_track)}:{len(cursor_track)}:"
            f"{value_track_label}"
        )
        return False, value

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        self.events.append(f"checkbox:{label}")
        return False, value

    def text_input(self, label: str, value: str) -> tuple[bool, str]:
        self.events.append(f"text_input:{label}:{value}")
        return False, value

    def select(
        self, label: str, selected_index: int, choices: list[str]
    ) -> tuple[bool, int]:
        self.events.append(f"select:{label}:{selected_index}:{','.join(choices)}")
        return False, selected_index

    def button(self, label: str) -> bool:
        self.events.append(f"button:{label}")
        return False

    def selectable(self, label: str, selected: bool = False) -> bool:
        self.events.append(f"selectable:{label}:{selected}")
        return False

    def same_line(self) -> None:
        self.events.append("same_line")

    def item_tooltip(self, text: str) -> None:
        self.events.append(f"tooltip:{text}")

    def plot_lines(self, label: str, values: list[float]) -> None:
        self.events.append(f"plot:{label}:{len(values)}")

    def separator(self) -> None:
        self.events.append("separator")

    def collapsing_header(self, label: str, default_open: bool = False) -> bool:
        self.events.append(f"collapsing:{label}:{default_open}")
        return True

    def begin_table(self, label: str, columns: list[str]) -> bool:
        self.events.append(f"table:{label}:{','.join(columns)}")
        return True

    def table_next_row(self) -> None:
        self.events.append("table_row")

    def table_next_column(self) -> bool:
        self.events.append("table_column")
        return True

    def end_table(self) -> None:
        self.events.append("end_table")


class _ScriptedPanelBuilder(_FakePanelBuilder):
    def __init__(
        self,
        *,
        clicked_buttons: set[str] | None = None,
        selected_items: set[str] | None = None,
        checkbox_values: dict[str, bool] | None = None,
        text_input_values: dict[str, str] | None = None,
        select_values: dict[str, int] | None = None,
        slider_values: dict[str, float] | None = None,
        timeline_values: dict[str, float] | None = None,
    ) -> None:
        super().__init__()
        self.clicked_buttons = clicked_buttons or set()
        self.selected_items = selected_items or set()
        self.checkbox_values = checkbox_values or {}
        self.text_input_values = text_input_values or {}
        self.select_values = select_values or {}
        self.slider_values = slider_values or {}
        self.timeline_values = timeline_values or {}

    def button(self, label: str) -> bool:
        self.events.append(f"button:{label}")
        return label in self.clicked_buttons

    def selectable(self, label: str, selected: bool = False) -> bool:
        self.events.append(f"selectable:{label}:{selected}")
        return label in self.selected_items

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        self.events.append(f"checkbox:{label}")
        if label in self.checkbox_values:
            return True, self.checkbox_values[label]
        return False, value

    def text_input(self, label: str, value: str) -> tuple[bool, str]:
        self.events.append(f"text_input:{label}:{value}")
        if label in self.text_input_values:
            return True, self.text_input_values[label]
        return False, value

    def select(
        self, label: str, selected_index: int, choices: list[str]
    ) -> tuple[bool, int]:
        self.events.append(f"select:{label}:{selected_index}:{','.join(choices)}")
        if label in self.select_values:
            return True, self.select_values[label]
        return False, selected_index

    def slider(
        self, label: str, value: float, minimum: float, maximum: float
    ) -> tuple[bool, float]:
        self.events.append(f"slider:{label}:{minimum}:{maximum}")
        if label in self.slider_values:
            return True, self.slider_values[label]
        return False, value

    def timeline(
        self,
        label: str,
        value: float,
        minimum: float,
        maximum: float,
        value_track: list[float] | tuple[float, ...] = (),
        marker_track: list[float] | tuple[float, ...] = (),
        cursor_track: list[float] | tuple[float, ...] = (),
        value_track_label: str = "Values",
    ) -> tuple[bool, float]:
        self.timelines.append(
            {
                "cursor_track": list(cursor_track),
                "label": label,
                "marker_track": list(marker_track),
                "value_track": list(value_track),
                "value_track_label": value_track_label,
            }
        )
        self.events.append(
            f"timeline:{label}:{minimum}:{maximum}:"
            f"{len(value_track)}:{len(marker_track)}:{len(cursor_track)}:"
            f"{value_track_label}"
        )
        if label in self.timeline_values:
            return True, self.timeline_values[label]
        return False, value


def test_planned_world_port_panels_expose_actionable_routes() -> None:
    _require_simulation_symbols("World")
    scenes = (
        planned.INVERSE_KINEMATICS,
        planned.SIMBICON_WALKING,
        planned.OPERATIONAL_SPACE_CONTROL,
        planned.MOBILE_MANIPULATION,
        robot_puppets.G1_PUPPET,
    )

    for scene in scenes:
        setup = scene.build()
        assert setup.info["planned_status"] == "planned World demo"
        assert setup.info["planned_world_port"] == scene.id
        assert setup.info["legacy_seeds"]
        for key in ("current_route", "target", "unblocker", "retire_when"):
            assert isinstance(setup.info[key], str), (scene.id, key)
            assert setup.info[key], (scene.id, key)

        builder = _FakePanelBuilder()
        setup.panels[0].build(builder, object())

        assert "text:status: planned World demo" in builder.events
        assert any(
            event.startswith("text:try now: ") for event in builder.events
        ), scene.id
        assert any(
            event.startswith("text:blocked on: ") for event in builder.events
        ), scene.id
        assert any(
            event.startswith("text:replace when: ") for event in builder.events
        ), scene.id


def test_high_value_world_scenes_expose_custom_panels() -> None:
    sx = _require_simulation_symbols("World")

    cases = [
        (articulated, "Articulated"),
        (floating_base, "Floating Base"),
        (contact, "Rigid Link Contact"),
        (rigid_body, "Rigid Bodies"),
        (rigid_body_modes, "Rigid Body Modes"),
        (rigid_free_flight, "Rigid Free Flight"),
        (rigid_frame_hierarchy, "Rigid Frame Hierarchy"),
        (rigid_external_loads, "Rigid External Loads"),
        (rigid_link_point_loads, "Rigid Link Point Loads"),
        (rigid_timestep_sensitivity, "Rigid Time Step Sensitivity"),
        (rigid_step_diagnostics, "Rigid Step Diagnostics"),
        (rigid_contact_scale_budget, "Rigid Contact Scale Budget"),
        (rigid_restitution_ladder, "Rigid Restitution Ladder"),
        (rigid_material_mixing, "Rigid Material Mixing"),
        (rigid_contact_inspector, "Rigid Contact Inspector"),
        (rigid_collision_query_options, "Rigid Collision Query Options"),
        (rigid_collision_casts, "Rigid Collision Casts"),
        (rigid_contact_manipulation, "Rigid Contact Manipulation"),
        (rigid_kinematic_driver, "Rigid Kinematic Driver"),
        (rigid_kinematic_normal_push, "Rigid Kinematic Normal Push"),
        (rigid_contact_solver_compare, "Rigid Contact Solver Compare"),
        (rigid_solver_compare, "Rigid Solver Compare"),
        (rigid_executor_equivalence, "Rigid Executor Equivalence"),
        (rigid_friction_threshold, "Rigid Friction Threshold"),
        (rigid_spin_roll_coupling, "Rigid Spin/Roll Coupling"),
        (rigid_stack_stability, "Rigid Stack Stability"),
        (rigid_ipc, "Rigid IPC Contact"),
        (rigid_ipc_edge_drop, "Rigid IPC Edge Drop"),
        (rigid_ipc_incline, "Rigid IPC Incline"),
        (rigid_ipc_pile, "Rigid IPC Pile"),
        (rigid_ipc_tunnel, "Rigid IPC Tunnel"),
        (rigid_ipc_stack_packet, "Rigid IPC Stack Packet"),
        (atlas_simbicon, "Atlas SIMBICON"),
        (variational_chain, "Variational Chain"),
        (variational_tumbler, "Variational Tumbler"),
    ]
    if hasattr(sx.World(), "add_rigid_body_fixed_joint"):
        cases.insert(4, (rigid_fixed_joint, "Rigid Fixed Joint"))
        cases.insert(5, (rigid_joint_breakage, "Rigid Joint Breakage"))
    if hasattr(sx.World(), "add_rigid_body_distance_spring"):
        cases.insert(6, (rigid_distance_spring, "Rigid Distance Spring"))
    if hasattr(sx.World(), "add_rigid_body_revolute_joint") and hasattr(
        sx.World(), "add_rigid_body_prismatic_joint"
    ):
        cases.insert(7, (rigid_limited_joints, "Rigid One-DOF Joints"))
    if hasattr(sx, "ActuatorType") and hasattr(sx, "JointSpec"):
        cases.insert(8, (rigid_joint_motor_limits, "Rigid Joint Motors & Limits"))
        cases.insert(
            9,
            (
                rigid_joint_passive_parameters,
                "Rigid Joint Passive Parameters",
            ),
        )
    if hasattr(sx, "JointType") and hasattr(sx, "JointSpec"):
        cases.insert(10, (rigid_screw_joint_pitch, "Rigid Screw Joint Pitch"))
        cases.insert(
            11,
            (
                rigid_multibody_dynamics_terms,
                "Rigid Multibody Dynamics Terms",
            ),
        )
        cases.insert(
            12,
            (
                rigid_link_center_of_mass,
                "Rigid Link Center of Mass",
            ),
        )
        cases.insert(13, (rigid_link_jacobian, "Rigid Link Jacobian"))
    if hasattr(sx, "LoopClosureSpec") and hasattr(sx, "ClosureDynamicsPolicy"):
        cases.insert(
            14,
            (
                rigid_multibody_solver_family,
                "Rigid Multibody Solver Family",
            ),
        )
        cases.insert(15, (rigid_loop_closure, "Rigid Loop Closure"))

    for scene_module, expected_title in cases:
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        assert setup.force_drag is not None
        assert [panel.title for panel in setup.panels] == [expected_title]

        setup.panels[0].build(builder, object())

        assert any(event.startswith("plot:") for event in builder.events)
        assert "text:External force" in builder.events
        assert any(event.startswith("text:drag target: ") for event in builder.events)
        assert "checkbox:Enable external force" in builder.events


def test_rigid_comparison_panels_label_the_compared_axis() -> None:
    _require_simulation_symbols(
        "RigidBodySolver", "ContactSolverMethod", "World", "JointSpec"
    )

    cases = [
        (
            rigid_body_modes,
            (
                "text:comparison axis: rigid-body mode semantics",
                "text:held fixed: solver Sequential impulse | executor Sequential | gravity scale 0.55 | force 3.00 N | time step 4.0 ms",
                "text:mode flags: dynamic, static, and kinematic",
            ),
        ),
        (
            rigid_timestep_sensitivity,
            (
                "text:comparison axis: time-step multiplier",
                "text:held fixed: solver Sequential impulse | executor Sequential | gravity scale 1.00",
            ),
        ),
        (
            rigid_step_diagnostics,
            (
                "text:comparison axis: workload shape",
                "text:held fixed: solver Sequential impulse | executor Sequential | time step 4.0 ms",
            ),
        ),
        (
            rigid_contact_scale_budget,
            (
                "text:comparison axis: contact workload size",
                "text:held fixed: solver Sequential impulse | executor Sequential | friction 0.72",
            ),
        ),
        (
            rigid_solver_compare,
            (
                "text:comparison axis: rigid-body solver family",
                "text:solver pair: Sequential impulse vs IPC barrier",
                "text:shared executor: Sequential",
            ),
        ),
        (
            rigid_executor_equivalence,
            (
                "text:comparison axis: executor only",
                "text:same physics solver: Sequential impulse",
            ),
        ),
        (
            rigid_contact_solver_compare,
            (
                "text:comparison axis: contact solver method",
                "text:shared rigid-body solver: sequential impulse",
                "text:contact-policy pair: sequential impulses vs boxed LCP",
            ),
        ),
        (
            rigid_friction_threshold,
            (
                "text:comparison axis: friction relative to static threshold",
                "text:held fixed: solver IPC | executor Sequential | time step 5.0 ms",
            ),
        ),
        (
            rigid_spin_roll_coupling,
            (
                "text:comparison axis: spin/roll initial condition",
                "text:held fixed: solver Sequential impulse | executor Sequential | time step 4.0 ms",
            ),
        ),
        (
            rigid_stack_stability,
            (
                "text:comparison axis: rigid-body solver family under stack load",
                "text:held fixed: executor Sequential | top mass ratio 20.0 | friction 0.80 | time step 6.0 ms",
                "text:solver pair: Sequential impulse vs IPC barrier",
            ),
        ),
        (
            rigid_contact_manipulation,
            (
                "text:comparison axis: rigid pusher contact response",
                "text:held fixed: executor Sequential | shared table/goal | target mass 1.0 | time step 4.0 ms",
                "text:solver pair: Sequential impulse vs IPC barrier",
            ),
        ),
        (
            rigid_kinematic_driver,
            (
                "text:comparison axis: prescribed tangential contact response",
                "text:held fixed: executor Sequential | tangential kinematic support | box mass 1.0 | time step 4.0 ms",
                "text:solver lanes: IPC grip, IPC low-friction slip, Sequential impulse caveat",
            ),
        ),
        (
            rigid_kinematic_normal_push,
            (
                "text:comparison axis: prescribed normal contact response",
                "text:held fixed: executor Sequential | normal kinematic paddle | zero friction | time step 4.0 ms",
                "text:solver lanes: IPC normal, IPC heavy target, Sequential impulse",
            ),
        ),
        (
            rigid_fixed_joint,
            (
                "text:comparison axis: fixed relative transform recovery",
                "text:held fixed: sequential rigid joints | static base | payload mass 1.0 | offset 0.85 m | time step 5.0 ms",
            ),
        ),
        (
            rigid_joint_breakage,
            (
                "slider:Break force log10(N):-12.0:12.0",
                "text:comparison axis: fixed break-force lifecycle",
                "text:held fixed: AVBD rigid joints | static base | payload mass 1.0 | offset 0.62 m | time step 4.0 ms",
            ),
        ),
        (
            rigid_distance_spring,
            (
                "slider:Rest length:0.25:0.75",
                "slider:Soft stiffness:10.0:100.0",
                "slider:Stiff stiffness:80.0:320.0",
                "slider:Offset stiffness:40.0:220.0",
                "text:comparison axis: distance-spring response family",
                "text:held fixed: executor Sequential | sequential impulse + AVBD springs | rest length 0.45 m | payload mass 1.0 | time step 4.0 ms",
            ),
        ),
        (
            rigid_limited_joints,
            (
                "slider:Perturbation:0.0:0.35",
                "text:comparison axis: one-DOF joint constraint family",
                "text:held fixed: sequential rigid joints | static bases | z-axis joints | payload mass 1.0 | time step 5.0 ms",
            ),
        ),
        (
            rigid_joint_motor_limits,
            (
                "slider:Command speed:0.05:0.85",
                "slider:Velocity limit:0.05:0.5",
                "slider:Position upper limit:0.12:0.65",
                "slider:Requested force:2.0:22.0",
                "slider:Effort cap:1.0:9.0",
                "text:comparison axis: World multibody actuator/limit family",
                "text:held fixed: World multibody joints | x-axis prismatic rails + y-axis revolute stop | carriage mass 2.0 | time step 5.0 ms",
            ),
        ),
        (
            rigid_joint_passive_parameters,
            (
                "slider:Hold force:0.0:12.0",
                "slider:Armature drive force:0.0:18.0",
                "text:comparison axis: passive joint parameter family",
                "text:held fixed: World prismatic joints | gravity off | contacts off | link mass 2.0 | time step 4.0 ms",
            ),
        ),
        (
            rigid_screw_joint_pitch,
            (
                "slider:Pitch scale:0.02:0.6",
                "slider:Gravity scale:0.0:1.4",
                "slider:Moving mass:0.5:5.0",
                "slider:Axial inertia:0.03:0.8",
                "text:comparison axis: screw pitch coupling family",
                "text:held fixed: World screw joints | contacts off | z-axis screw | moving mass 2.0 | axial inertia 0.12 | time step 3.0 ms",
            ),
        ),
        (
            rigid_multibody_dynamics_terms,
            (
                "slider:Target acceleration:0.2:5.0",
                "slider:Joint impulse:0.2:8.0",
                "slider:Heavy distal mass scale:1.2:8.0",
                "slider:Gravity scale:0.0:1.5",
                "text:comparison axis: joint-space dynamics term family",
                "text:held fixed: World multibody dynamics | contacts off | fixed-base revolute links | target acceleration 2.2 | impulse 3.0 | gravity scale 1.0 | time step 3.0 ms",
            ),
        ),
        (
            rigid_link_center_of_mass,
            (
                "slider:COM offset:0.0:0.32",
                "slider:Gravity scale:0.0:1.5",
                "slider:Link mass:0.5:5.0",
                "slider:High-inertia multiplier:1.0:8.0",
                "text:comparison axis: link center-of-mass offset family",
                "text:held fixed: World multibody revolute links | contacts off | visual geometry fixed | link mass 2.0 | gravity scale 1.0 | time step 3.0 ms",
            ),
        ),
        (
            rigid_link_jacobian,
            (
                "slider:Motion speed:0.1:2.4",
                "slider:Elbow phase:-3.14:3.14",
                "slider:Wrench force:0.0:3.0",
                "slider:Wrench angle:-120.0:120.0",
                "slider:Wrench moment:-0.6:0.6",
                "text:comparison axis: link-origin Jacobian mapping family",
                "text:held fixed: World multibody link Jacobian | contacts off | gravity off | two revolute links | link length 0.55 | time step 4.0 ms",
            ),
        ),
        (
            rigid_multibody_solver_family,
            (
                "slider:Gravity scale:0.0:1.8",
                "text:comparison axis: multibody integration solve-policy family",
                "text:held fixed: World multibody point closure | contacts off | three revolute links | link length 0.55 | gravity scale 1.0 | time step 5.0 ms",
            ),
        ),
        (
            rigid_loop_closure,
            (
                "slider:Gravity scale:0.0:1.4",
                "text:comparison axis: loop-closure family and solve policy",
                "text:held fixed: Variational rigid multibody | contacts off | four revolute links | link length 0.56 | gravity scale 1.0 | time step 5.0 ms",
                "text:executor: Sequential",
            ),
        ),
    ]

    for scene_module, expected_events in cases:
        setup = scene_module.build()
        builder = _FakePanelBuilder()

        setup.panels[0].build(builder, object())

        for event in expected_events:
            assert event in builder.events


@pytest.mark.parametrize(
    ("scene_module", "controller_key"),
    [
        (rigid_body_modes, "rigid_body_modes_controller"),
        (rigid_free_flight, "rigid_free_flight_controller"),
        (rigid_frame_hierarchy, "rigid_frame_hierarchy_controller"),
        (rigid_timestep_sensitivity, "rigid_timestep_sensitivity_controller"),
        (rigid_step_diagnostics, "rigid_step_diagnostics_controller"),
        (rigid_contact_scale_budget, "rigid_contact_scale_budget_controller"),
        (rigid_solver_compare, "rigid_solver_compare_controller"),
        (rigid_restitution_ladder, "rigid_restitution_ladder_controller"),
        (rigid_material_mixing, "rigid_material_mixing_controller"),
        (rigid_contact_solver_compare, "rigid_contact_solver_compare_controller"),
        (rigid_friction_threshold, "rigid_friction_threshold_controller"),
        (rigid_spin_roll_coupling, "rigid_spin_roll_coupling_controller"),
        (rigid_stack_stability, "rigid_stack_stability_controller"),
        (contact, "rigid_link_contact_controller"),
        (rigid_contact_manipulation, "rigid_contact_manipulation_controller"),
        (rigid_kinematic_driver, "rigid_kinematic_driver_controller"),
        (rigid_kinematic_normal_push, "rigid_kinematic_normal_push_controller"),
        (rigid_external_loads, "rigid_external_loads_controller"),
        (rigid_link_point_loads, "rigid_link_point_loads_controller"),
        (rigid_distance_spring, "rigid_distance_spring_controller"),
        (
            rigid_joint_passive_parameters,
            "rigid_joint_passive_parameters_controller",
        ),
        (rigid_screw_joint_pitch, "rigid_screw_joint_pitch_controller"),
        (rigid_multibody_dynamics_terms, "rigid_multibody_dynamics_terms_controller"),
        (rigid_link_center_of_mass, "rigid_link_center_of_mass_controller"),
    ],
)
def test_rigid_executor_panel_edits_reset_visual_runs(
    scene_module: object, controller_key: str
) -> None:
    _require_simulation_symbols(
        "RigidBodySolver", "ContactSolverMethod", "World", "JointSpec"
    )

    setup = scene_module.build()
    controller = setup.info[controller_key]
    if len(controller._executors) < 2:
        pytest.skip("executor alternatives unavailable in this build")
    assert setup.pre_step is not None

    world = setup.info["sx_world"]
    setup.pre_step()
    assert world.time > 0.0

    target_executor = len(controller._executors) - 1
    builder = _ScriptedPanelBuilder(select_values={"Executor": target_executor})
    setup.panels[0].build(builder, object())

    assert any(event.startswith("select:Executor:") for event in builder.events)
    assert controller.executor_index == target_executor
    assert world.time == pytest.approx(0.0)

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    expected_executor = controller._executors[target_executor][0]
    assert capture_metrics["executor"] == expected_executor
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        float(target_executor)
    )


@pytest.mark.parametrize(
    ("scene_module", "controller_key", "select_label"),
    [
        (rigid_body, "rigid_body_controller", "Solver"),
        (rigid_body_modes, "rigid_body_modes_controller", "Solver"),
        (rigid_timestep_sensitivity, "rigid_timestep_sensitivity_controller", "Solver"),
        (rigid_step_diagnostics, "rigid_step_diagnostics_controller", "Solver"),
        (rigid_contact_scale_budget, "rigid_contact_scale_budget_controller", "Solver"),
        (
            rigid_executor_equivalence,
            "rigid_executor_equivalence_controller",
            "Physics solver",
        ),
        (rigid_restitution_ladder, "rigid_restitution_ladder_controller", "Solver"),
    ],
)
def test_rigid_solver_panel_edits_reset_visual_runs(
    scene_module: object, controller_key: str, select_label: str
) -> None:
    _require_simulation_symbols("RigidBodySolver", "World")

    setup = scene_module.build()
    controller = setup.info[controller_key]
    if len(getattr(scene_module, "_SOLVERS", ())) < 2:
        pytest.skip("solver alternatives unavailable in this build")
    assert setup.pre_step is not None

    world = setup.info["sx_world"]
    setup.pre_step()
    assert world.time > 0.0

    target_solver = 1
    builder = _ScriptedPanelBuilder(select_values={select_label: target_solver})
    setup.panels[0].build(builder, object())

    assert any(event.startswith(f"select:{select_label}:") for event in builder.events)
    assert controller.solver_index == target_solver
    assert world.time == pytest.approx(0.0)

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["controls"]["solver_index"] == pytest.approx(
        float(target_solver)
    )


def test_rigid_collision_query_options_panel_edits_capture_controls() -> None:
    _require_simulation_symbols("World", "CollisionQueryOptions")

    setup = rigid_collision_query_options.build()
    controller = setup.info["rigid_collision_query_options_controller"]
    builder = _ScriptedPanelBuilder(
        checkbox_values={
            "Rigid body pairs": False,
            "Same-multibody link pairs": False,
        },
        select_values={"Ignored pair": 2},
    )

    setup.panels[0].build(builder, object())

    assert "checkbox:Rigid body pairs" in builder.events
    assert "checkbox:Same-multibody link pairs" in builder.events
    assert any(event.startswith("select:Ignored pair:") for event in builder.events)
    assert controller.include_rigid_body_pairs is False
    assert controller.include_rigid_body_link_pairs is True
    assert controller.include_link_pairs is True
    assert controller.include_same_multibody_link_pairs is False
    assert controller.ignored_pair_key == "rigid_link"

    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    controls = capture_metrics["controls"]
    assert controls["include_rigid_body_pairs"] is False
    assert controls["include_rigid_body_link_pairs"] is True
    assert controls["include_link_pairs"] is True
    assert controls["include_same_multibody_link_pairs"] is False
    assert controls["ignored_pair_key"] == "rigid_link"
    assert controls["ignored_pair_index"] == 2
    assert capture_metrics["active_contact_count"] == 1
    assert capture_metrics["option_filtered_contact_count"] == 2
    assert capture_metrics["ignored_contact_count"] == 1


@pytest.mark.parametrize(
    "scene_module",
    [rigid_ipc, rigid_ipc_slide, rigid_ipc_incline, rigid_ipc_pile],
)
def test_rigid_ipc_shelf_panel_edits_capture_controls(scene_module: object) -> None:
    _require_simulation_symbols("RigidBodySolver")

    setup = scene_module.build()
    builder = _ScriptedPanelBuilder(slider_values={"Friction": 0.23})
    setup.panels[0].build(builder, object())

    assert "slider:Friction:0.0:1.0" in builder.events
    capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert capture_metrics["controls"]["friction"] == pytest.approx(0.23)
    assert capture_metrics["friction"] == pytest.approx(0.23)


def test_world_related_shelf_panel_edits_capture_controls() -> None:
    _require_simulation_symbols("World")

    floating_setup = floating_base.build()
    floating_builder = _ScriptedPanelBuilder(
        slider_values={"Spin command": 4.5}
    )
    floating_setup.panels[0].build(floating_builder, object())
    floating_metrics = floating_setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert "slider:Spin command:-6.0:6.0" in floating_builder.events
    assert floating_metrics["controls"]["spin_command"] == pytest.approx(4.5)
    assert floating_metrics["spin_command"] == pytest.approx(4.5)

    articulated_setup = articulated.build()
    articulated_builder = _ScriptedPanelBuilder(
        slider_values={
            "Shoulder damping": 1.25,
            "Wrist damping": 1.5,
        }
    )
    articulated_setup.panels[0].build(articulated_builder, object())
    articulated_metrics = articulated_setup.info[CAPTURE_METRICS_INFO_KEY]()
    assert "slider:Shoulder damping:0.0:2.0" in articulated_builder.events
    assert "slider:Wrist damping:0.0:2.0" in articulated_builder.events
    assert articulated_metrics["controls"]["shoulder_damping"] == pytest.approx(
        1.25
    )
    assert articulated_metrics["controls"]["wrist_damping"] == pytest.approx(
        1.5
    )
    assert articulated_metrics["shoulder_damping"] == pytest.approx(1.25)
    assert articulated_metrics["wrist_damping"] == pytest.approx(1.5)


def test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals() -> None:
    _require_simulation_symbols("RigidBodySolver")

    cases = [
        (
            rigid_ipc_stack_packet.build,
            "rigid_ipc_stack_packet",
            "text:capture-first rigid IPC stack",
        ),
        (
            rigid_ipc_stack_packet.build_heavy,
            "rigid_ipc_heavy_stack_packet",
            "text:capture-first heavy rigid IPC stack",
        ),
    ]
    for build_scene, scene_id, heading in cases:
        setup = build_scene()
        controller = setup.info[f"{scene_id}_controller"]
        setup.pre_step()

        builder = _ScriptedPanelBuilder(
            slider_values={
                "Friction": 0.42,
                "Frame budget ms": 44.0,
            }
        )
        setup.panels[0].build(builder, _FakePanelContext())

        assert setup.info[f"{scene_id}_capture_first"] is True
        assert setup.info[f"{scene_id}_benchmark"] == "bm_rigid_ipc_solver"
        assert setup.info["rigid_ipc_stack_packet_variant"] == scene_id
        assert heading in builder.events
        assert "text:not a numbered World Rigid Body workflow row" in builder.events
        assert "text:benchmark owner: bm_rigid_ipc_solver" in builder.events
        assert any(event.startswith("text:top mass:") for event in builder.events)
        assert any(event.startswith("plot:Step wall ms:") for event in builder.events)
        assert any(event.startswith("plot:Min clearance:") for event in builder.events)
        assert any(event.startswith("plot:Contact count:") for event in builder.events)
        capture_metrics = setup.info[CAPTURE_METRICS_INFO_KEY]()
        assert capture_metrics["controls"]["friction"] == pytest.approx(0.42)
        assert capture_metrics["controls"]["frame_budget_ms"] == pytest.approx(44.0)
        assert controller._last_metrics


def test_rigid_workflow_panel_renders_guidance_for_numbered_rows() -> None:
    def build_guidance_events(scene_id: str) -> list[str]:
        scene = PythonDemoScene(
            id=scene_id,
            title=f"Test {scene_id}",
            category="World Rigid Body",
            summary="Workflow guidance test.",
            build=lambda: SceneSetup(),
        )
        _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()
        assert panels is not None
        workflow_panels = [panel for panel in panels if panel.title == "Rigid Workflow"]
        assert len(workflow_panels) == 1
        builder = _FakePanelBuilder()
        workflow_panels[0].build(builder, _FakePanelContext())
        return builder.events

    cases = (
        ("rigid_body", "Previous: start", "Next: 02/36 Body modes"),
        (
            "rigid_solver_compare",
            "Previous: 14/36 Collision casts",
            "Next: 16/36 Executor equivalence",
        ),
        (
            "rigid_loop_closure",
            "Previous: 35/36 Multibody solver",
            "Next: done",
        ),
    )

    for scene_id, previous_text, next_text in cases:
        guide = RIGID_VISUAL_WORKFLOW_GUIDES[scene_id]
        events = build_guidance_events(scene_id)

        assert f"text:{guide.index:02d}/{guide.count:02d} {guide.label}" in events
        assert "text:Question" in events
        assert f"text:{guide.question}" in events
        assert "text:Try first" in events
        assert f"text:{guide.try_first}" in events
        assert "text:Look for" in events
        for signal in guide.inspect:
            assert f"text:{signal}" in events
        assert f"text:{guide.healthy_signal}" in events
        assert "text:Do not infer" in events
        assert f"text:{guide.scope}" in events
        assert "text:Capture evidence" in events
        assert (
            "text:"
            f"{guide.capture_frames} frames | "
            f"{guide.capture_width}x{guide.capture_height} | docked UI"
        ) in events
        assert (
            "text:"
            + _rigid_workflow_viewer_command(
                guide.scene_id, guide.capture_width, guide.capture_height
            )
        ) in events
        assert (
            "tooltip:Open this row live in py-demos for interactive debugging."
            in events
        )
        assert f"text:{guide.capture_command}" in events
        assert "tooltip:Run from the repository root to regenerate this row." in events
        assert "text:Review packet" in events
        assert (
            "text:"
            + _rigid_workflow_packet_command(
                output_dir="/tmp/dart_capture_rigid_workflow"
            )
        ) in events
        assert (
            "tooltip:Capture all numbered rows and write review_index.html."
            in events
        )
        assert f"text:{_rigid_workflow_row_packet_command(guide)}" in events
        assert (
            "tooltip:Capture only this workflow row in a review packet."
            in events
        )
        assert f"text:{_rigid_workflow_row_video_packet_command(guide)}" in events
        assert (
            "tooltip:Capture this row as a review packet with PNG frames "
            "and MP4 motion."
        ) in events
        assert (
            "text:"
            + _rigid_workflow_packet_command(
                include_related=True,
                include_ipc_shelf=True,
                include_packets=True,
                output_dir="/tmp/dart_capture_rigid_workflow_extended",
            )
        ) in events
        assert (
            "tooltip:Capture numbered rows plus related, Rigid IPC shelf, "
            "and packet rows."
        ) in events
        assert (
            "text:"
            + _rigid_workflow_packet_command(
                include_related=True,
                include_ipc_shelf=True,
                include_packets=True,
                continue_on_failure=True,
                output_dir="/tmp/dart_capture_rigid_workflow_resilient",
            )
        ) in events
        assert (
            "tooltip:Capture the extended packet while preserving later-row "
            "evidence after a row fails."
        ) in events
        assert "text:Route" in events
        if guide.previous_scene_id is None:
            assert f"text:{previous_text}" in events
        else:
            assert (
                f"selectable:{previous_text}##rigid_workflow_previous:False"
                in events
            )
        if guide.next_scene_id is None:
            assert f"text:{next_text}" in events
        else:
            assert f"selectable:{next_text}##rigid_workflow_next:False" in events
        assert (
            "selectable:"
            f"Restart row: {guide.index:02d}/{guide.count:02d} {guide.label}"
            "##rigid_workflow_restart:False"
        ) in events
        assert any(event.startswith("select:Jump to row:") for event in events)
        assert "text_input:Find row:" in events


def test_rigid_workflow_panel_route_rows_request_scene_switches() -> None:
    scene = PythonDemoScene(
        id="rigid_solver_compare",
        title="Test rigid_solver_compare",
        category="World Rigid Body",
        summary="Workflow route test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()

    previous_label = "Previous: 14/36 Collision casts##rigid_workflow_previous"
    next_label = "Next: 16/36 Executor equivalence##rigid_workflow_next"
    replay_label = "Restart row: 15/36 Solver family##rigid_workflow_restart"
    workflow_panel.build(
        _ScriptedPanelBuilder(selected_items={previous_label}),
        context,
    )
    workflow_panel.build(
        _ScriptedPanelBuilder(selected_items={next_label}),
        context,
    )
    workflow_panel.build(
        _ScriptedPanelBuilder(selected_items={replay_label}),
        context,
    )

    assert context.scene_switch_requests == [
        "rigid_collision_casts",
        "rigid_executor_equivalence",
    ]
    assert context.scene_replay_requests == ["rigid_solver_compare"]


def test_rigid_workflow_panel_related_evidence_routes_to_other_shelves() -> None:
    free_flight_scene = PythonDemoScene(
        id="rigid_free_flight",
        title="Test rigid_free_flight",
        category="World Rigid Body",
        summary="Related floating-base route test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(
        free_flight_scene
    )()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    floating_label = (
        "Related shelf: World Rigid Body / floating_base - "
        "broader floating-joint row"
        "##rigid_workflow_related_floating_base"
    )
    floating_builder = _ScriptedPanelBuilder(selected_items={floating_label})

    workflow_panel.build(floating_builder, context)

    assert (
        "selectable:Related shelf: World Rigid Body / floating_base - "
        "broader floating-joint row"
        "##rigid_workflow_related_floating_base:False"
    ) in floating_builder.events
    assert any(
        event.startswith(
            "tooltip:Open floating_base from the World Rigid Body shelf."
        )
        for event in floating_builder.events
    )
    assert context.scene_switch_requests == ["floating_base"]

    solver_scene = PythonDemoScene(
        id="rigid_solver_compare",
        title="Test rigid_solver_compare",
        category="World Rigid Body",
        summary="Related evidence route test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(solver_scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    tunnel_label = (
        "Related shelf: Rigid IPC / rigid_ipc_tunnel - focused no-tunneling view"
        "##rigid_workflow_related_rigid_ipc_tunnel"
    )
    tunnel_builder = _ScriptedPanelBuilder(selected_items={tunnel_label})

    workflow_panel.build(tunnel_builder, context)

    assert "text:Related evidence" in tunnel_builder.events
    assert (
        "selectable:Related shelf: Rigid IPC / rigid_ipc_tunnel - "
        "focused no-tunneling view"
        "##rigid_workflow_related_rigid_ipc_tunnel:False"
    ) in tunnel_builder.events
    assert any(
        event.startswith(
            "tooltip:Open rigid_ipc_tunnel from the Rigid IPC shelf."
        )
        for event in tunnel_builder.events
    )
    assert context.scene_switch_requests == ["rigid_ipc_tunnel"]

    context = _FakePanelContext()
    edge_drop_label = (
        "Related shelf: Rigid IPC / rigid_ipc_edge_drop - "
        "degenerate edge-contact view"
        "##rigid_workflow_related_rigid_ipc_edge_drop"
    )
    edge_drop_builder = _ScriptedPanelBuilder(selected_items={edge_drop_label})

    workflow_panel.build(edge_drop_builder, context)

    assert (
        "selectable:Related shelf: Rigid IPC / rigid_ipc_edge_drop - "
        "degenerate edge-contact view"
        "##rigid_workflow_related_rigid_ipc_edge_drop:False"
    ) in edge_drop_builder.events
    assert any(
        event.startswith(
            "tooltip:Open rigid_ipc_edge_drop from the Rigid IPC shelf."
        )
        for event in edge_drop_builder.events
    )
    assert context.scene_switch_requests == ["rigid_ipc_edge_drop"]

    contact_policy_scene = PythonDemoScene(
        id="rigid_contact_solver_compare",
        title="Test rigid_contact_solver_compare",
        category="World Rigid Body",
        summary="Related differentiable route test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(
        contact_policy_scene
    )()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    diff_label = (
        "Related shelf: Differentiable / diff_drone_liftoff - "
        "contact-gradient route"
        "##rigid_workflow_related_diff_drone_liftoff"
    )
    diff_builder = _ScriptedPanelBuilder(selected_items={diff_label})

    workflow_panel.build(diff_builder, context)

    assert (
        "selectable:Related shelf: Differentiable / diff_drone_liftoff - "
        "contact-gradient route"
        "##rigid_workflow_related_diff_drone_liftoff:False"
    ) in diff_builder.events
    assert any(
        event.startswith(
            "tooltip:Open diff_drone_liftoff from the Differentiable shelf."
        )
        for event in diff_builder.events
    )
    assert context.scene_switch_requests == ["diff_drone_liftoff"]

    context = _FakePanelContext()
    pre_contact_label = (
        "Related shelf: Differentiable / diff_pre_contact_surrogate - "
        "pre-contact gradient route"
        "##rigid_workflow_related_diff_pre_contact_surrogate"
    )
    pre_contact_builder = _ScriptedPanelBuilder(
        selected_items={pre_contact_label}
    )

    workflow_panel.build(pre_contact_builder, context)

    assert (
        "selectable:Related shelf: Differentiable / diff_pre_contact_surrogate - "
        "pre-contact gradient route"
        "##rigid_workflow_related_diff_pre_contact_surrogate:False"
    ) in pre_contact_builder.events
    assert any(
        event.startswith(
            "tooltip:Open diff_pre_contact_surrogate from the Differentiable shelf."
        )
        for event in pre_contact_builder.events
    )
    assert context.scene_switch_requests == ["diff_pre_contact_surrogate"]

    motor_scene = PythonDemoScene(
        id="rigid_joint_motor_limits",
        title="Test rigid_joint_motor_limits",
        category="World Rigid Body",
        summary="Related AVBD motor route test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(motor_scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    revolute_label = (
        "Related shelf: AVBD Rigid Constraints (sx) / "
        "avbd_rigid_revolute_motor - free-rigid hinge motor"
        "##rigid_workflow_related_avbd_rigid_revolute_motor"
    )
    prismatic_label = (
        "Related shelf: AVBD Rigid Constraints (sx) / "
        "avbd_rigid_prismatic_motor - free-rigid slider motor"
        "##rigid_workflow_related_avbd_rigid_prismatic_motor"
    )
    motor_builder = _ScriptedPanelBuilder(selected_items={prismatic_label})

    workflow_panel.build(motor_builder, context)

    assert f"selectable:{revolute_label}:False" in motor_builder.events
    assert f"selectable:{prismatic_label}:False" in motor_builder.events
    assert any(
        event.startswith(
            "tooltip:Open avbd_rigid_prismatic_motor from the "
            "AVBD Rigid Constraints (sx) shelf."
        )
        for event in motor_builder.events
    )
    assert context.scene_switch_requests == ["avbd_rigid_prismatic_motor"]


def test_rigid_workflow_panel_jump_selector_requests_scene_switch() -> None:
    scene = PythonDemoScene(
        id="rigid_solver_compare",
        title="Test rigid_solver_compare",
        category="World Rigid Body",
        summary="Workflow jump test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()

    workflow_panel.build(
        _ScriptedPanelBuilder(select_values={"Jump to row": 7}),
        context,
    )

    assert context.scene_switch_requests == ["rigid_step_diagnostics"]


def test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch() -> None:
    scene = PythonDemoScene(
        id="rigid_solver_compare",
        title="Test rigid_solver_compare",
        category="World Rigid Body",
        summary="Workflow search test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    target_label = (
        "19/36 Friction threshold - rigid_friction_threshold"
        "##rigid_workflow_find_rigid_friction_threshold"
    )
    builder = _ScriptedPanelBuilder(
        text_input_values={"Find row": "stick/slip"},
        selected_items={target_label},
    )

    workflow_panel.build(builder, context)

    assert (
        "selectable:"
        "19/36 Friction threshold - rigid_friction_threshold"
        "##rigid_workflow_find_rigid_friction_threshold:False"
    ) in builder.events
    assert "tooltip:Where is the inclined-ramp stick/slip boundary?" in builder.events
    assert context.scene_switch_requests == ["rigid_friction_threshold"]


def test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats() -> None:
    contact_ids = [guide.scene_id for guide in _workflow_matching_guides("contact")]
    solver_ids = [guide.scene_id for guide in _workflow_matching_guides("solver")]
    sequential_impulse_ids = [
        guide.scene_id for guide in _workflow_matching_guides("sequential impulse")
    ]
    si_ids = [guide.scene_id for guide in _workflow_matching_guides("SI")]

    assert contact_ids[:4] == [
        "contact",
        "rigid_contact_scale_budget",
        "rigid_contact_inspector",
        "rigid_contact_solver_compare",
    ]
    assert not {
        "rigid_body",
        "rigid_body_modes",
        "rigid_free_flight",
        "rigid_frame_hierarchy",
    }.intersection(contact_ids)
    assert solver_ids[:3] == [
        "rigid_solver_compare",
        "rigid_contact_solver_compare",
        "rigid_multibody_solver_family",
    ]
    assert "rigid_body" not in solver_ids[:3]
    assert sequential_impulse_ids[:2] == [
        "rigid_solver_compare",
        "rigid_contact_solver_compare",
    ]
    assert "rigid_joint_breakage" not in sequential_impulse_ids[:2]
    assert si_ids[:1] == ["rigid_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("RigidBodySolver")
    ][:1] == ["rigid_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("rigid-body solver")
    ][:1] == ["rigid_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("solver method")
    ][:1] == ["rigid_solver_compare"]


def test_rigid_workflow_search_finds_backend_and_profile_aliases() -> None:
    assert [
        guide.scene_id for guide in _workflow_matching_guides("step profile")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("accelerated backend")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("backend status")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("memory diagnostics")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("worker count")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("backend comparison")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("parallel backend")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("compute backend")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("backend/executor")
    ][:1] == ["rigid_step_diagnostics"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("compute executor")
    ][:1] == ["rigid_executor_equivalence"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("executor comparison")
    ][:1] == ["rigid_executor_equivalence"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("same physics solver")
    ][:1] == ["rigid_executor_equivalence"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("taskflow executor")
    ][:1] == ["rigid_executor_equivalence"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("threaded executor")
    ][:1] == ["rigid_executor_equivalence"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("contact solver policy")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("solver policy")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("lcp")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("ContactSolverMethod")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("contact-policy pair")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("contact model")
    ][:1] == ["rigid_contact_solver_compare"]


def test_rigid_workflow_search_routes_deferred_api_terms() -> None:
    assert [
        guide.scene_id for guide in _workflow_matching_guides("direct rigid body impulse")
    ][:1] == ["rigid_external_loads"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("velocity impulse")
    ][:1] == ["rigid_external_loads"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("sleep wake")
    ][:1] == ["rigid_body_modes"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("island activation")
    ][:1] == ["rigid_body_modes"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("loop closure compliance")
    ][:1] == ["rigid_loop_closure"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("closure stiffness")
    ][:1] == ["rigid_loop_closure"]

    assert "direct rigid-body impulse" in (
        RIGID_VISUAL_WORKFLOW_GUIDES["rigid_external_loads"].scope
    )
    assert "no sleep/wake or island activation API claim" in (
        RIGID_VISUAL_WORKFLOW_GUIDES["rigid_body_modes"].scope
    )
    assert "not a compliance" in (
        RIGID_VISUAL_WORKFLOW_GUIDES["rigid_loop_closure"].scope
    )


def test_rigid_workflow_guides_expose_capture_commands() -> None:
    assert len(RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS) == len(RIGID_VISUAL_WORKFLOW_GUIDES)
    for scene_id, frames, width, height, show_ui in RIGID_VISUAL_WORKFLOW_CAPTURE_SPECS:
        guide = RIGID_VISUAL_WORKFLOW_GUIDES[scene_id]
        assert guide.capture_frames == frames
        assert guide.capture_width == width
        assert guide.capture_height == height
        assert guide.capture_show_ui is show_ui
        expected_command = (
            "pixi run py-demo-capture -- "
            f"--scene {scene_id} --frames {frames} --width {width} --height {height}"
        )
        if show_ui:
            expected_command = f"{expected_command} --show-ui"
        assert guide.capture_command == expected_command


def test_rigid_workflow_search_finds_related_evidence_targets() -> None:
    assert [
        guide.scene_id for guide in _workflow_matching_guides("rigid_ipc_tunnel")
    ][:1] == ["rigid_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("rigid_ipc_edge_drop")
    ][:1] == ["rigid_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("floating_base")
    ][:1] == ["rigid_free_flight"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("two-link arm")
    ][:1] == ["rigid_multibody_dynamics_terms"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("contact gradient")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("pre-contact surrogate")
    ][:1] == ["rigid_contact_solver_compare"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("avbd fixed contact")
    ][:1] == ["contact"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("avbd spherical")
    ][:1] == ["rigid_joint_breakage"]
    assert [
        guide.scene_id for guide in _workflow_matching_guides("avbd prismatic")
    ][:1] == ["rigid_joint_motor_limits"]


def test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch() -> None:
    scene = PythonDemoScene(
        id="rigid_contact_inspector",
        title="Test rigid_contact_inspector",
        category="World Rigid Body",
        summary="Workflow row-id search test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    target_label = (
        "15/36 Solver family - rigid_solver_compare"
        "##rigid_workflow_find_rigid_solver_compare"
    )
    builder = _ScriptedPanelBuilder(
        text_input_values={"Find row": "15/36"},
        selected_items={target_label},
    )

    workflow_panel.build(builder, context)

    assert (
        "selectable:"
        "15/36 Solver family - rigid_solver_compare"
        "##rigid_workflow_find_rigid_solver_compare:False"
    ) in builder.events
    assert (
        f"tooltip:{RIGID_VISUAL_WORKFLOW_GUIDES['rigid_solver_compare'].question}"
        in builder.events
    )
    assert context.scene_switch_requests == ["rigid_solver_compare"]


def test_rigid_workflow_panel_opens_related_evidence_search_matches() -> None:
    scene = PythonDemoScene(
        id="rigid_solver_compare",
        title="Test rigid_solver_compare",
        category="World Rigid Body",
        summary="Workflow related-search test.",
        build=lambda: SceneSetup(),
    )
    _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()
    assert panels is not None
    workflow_panel = [panel for panel in panels if panel.title == "Rigid Workflow"][0]
    context = _FakePanelContext()
    target_label = (
        "29/36 Motor limits - rigid_joint_motor_limits "
        "(related: avbd_rigid_prismatic_motor)"
        "##rigid_workflow_find_rigid_joint_motor_limits"
    )
    builder = _ScriptedPanelBuilder(
        text_input_values={"Find row": "avbd prismatic"},
        selected_items={target_label},
    )

    workflow_panel.build(builder, context)

    assert (
        "selectable:"
        "29/36 Motor limits - rigid_joint_motor_limits "
        "(related: avbd_rigid_prismatic_motor)"
        "##rigid_workflow_find_rigid_joint_motor_limits:False"
    ) in builder.events
    assert any(
        event.startswith(
            "tooltip:Do joint motors and limits clamp commands? "
            "Related evidence match: AVBD Rigid Constraints (sx) / "
            "avbd_rigid_prismatic_motor:"
        )
        for event in builder.events
    )
    assert context.scene_switch_requests == ["avbd_rigid_prismatic_motor"]

    context = _FakePanelContext()
    edge_drop_label = (
        "15/36 Solver family - rigid_solver_compare "
        "(related: rigid_ipc_edge_drop)"
        "##rigid_workflow_find_rigid_solver_compare"
    )
    edge_drop_builder = _ScriptedPanelBuilder(
        text_input_values={"Find row": "rigid_ipc_edge_drop"},
        selected_items={edge_drop_label},
    )

    workflow_panel.build(edge_drop_builder, context)

    assert (
        "selectable:"
        "15/36 Solver family - rigid_solver_compare "
        "(related: rigid_ipc_edge_drop)"
        "##rigid_workflow_find_rigid_solver_compare:True"
    ) in edge_drop_builder.events
    assert any(
        event.startswith(
            "tooltip:How do the rigid method families differ visually? "
            "Related evidence match: Rigid IPC / rigid_ipc_edge_drop:"
        )
        for event in edge_drop_builder.events
    )
    assert context.scene_switch_requests == ["rigid_ipc_edge_drop"]


def test_rigid_workflow_panel_skips_non_numbered_world_rows() -> None:
    scene = PythonDemoScene(
        id="articulated",
        title="Articulated",
        category="World Rigid Body",
        summary="Not part of the numbered rigid verifier.",
        build=lambda: SceneSetup(),
    )

    _pre_step, _force_drag, panels, _provider = _make_world_factory(scene)()

    assert panels is None


def test_numbered_rigid_workflow_factory_combines_panels() -> None:
    _require_simulation_symbols("World")

    _pre_step, _force_drag, panels, _provider = _make_world_factory(
        rigid_contact_scale_budget.SCENE
    )()

    assert panels is not None
    assert [panel.title for panel in panels] == [
        "Rigid Workflow",
        "Rigid Contact Scale Budget",
        "Replay",
    ]


def test_rigid_body_panel_resets_baseline_scene() -> None:
    _require_simulation_symbols("World")
    setup = rigid_body.build()
    controller = setup.info["rigid_body_controller"]

    for _ in range(20):
        assert setup.pre_step is not None
        setup.pre_step()

    assert controller.world.time > 0.0
    builder = _ScriptedPanelBuilder(clicked_buttons={"Reset baseline scene"})
    setup.panels[0].build(builder, object())

    assert "button:Reset baseline scene" in builder.events
    assert controller.world.time == pytest.approx(0.0)
    assert len(controller._speed_history) == 1


def test_rigid_joint_breakage_panel_resets_lifecycle() -> None:
    _require_simulation_symbols("World")

    setup = rigid_joint_breakage.build()
    joint = setup.info["joint"]
    payload = setup.info["payload"]
    initial_payload = np.asarray(payload.translation, dtype=float).reshape(3)
    for _ in range(45):
        assert setup.pre_step is not None
        setup.pre_step()

    assert joint.is_broken

    builder = _ScriptedPanelBuilder(
        clicked_buttons={"Reset breakage lifecycle"},
    )
    setup.panels[0].build(builder, object())

    assert "button:Reset breakage lifecycle" in builder.events
    assert not joint.is_broken
    reset_payload = np.asarray(payload.translation, dtype=float).reshape(3)
    assert reset_payload.tolist() == pytest.approx(initial_payload.tolist())


def test_rigid_joint_breakage_panel_edits_break_force_threshold() -> None:
    _require_simulation_symbols("World")

    setup = rigid_joint_breakage.build()
    joint = setup.info["joint"]

    builder = _ScriptedPanelBuilder(
        slider_values={"Break force log10(N)": 2.0},
        clicked_buttons={"Reset with current threshold"},
    )
    setup.panels[0].build(builder, object())

    assert "slider:Break force log10(N):-12.0:12.0" in builder.events
    assert "button:Reset with current threshold" in builder.events
    assert joint.break_force == pytest.approx(100.0)
    capture_metrics = setup.info["capture_metrics"]()
    assert capture_metrics["controls"]["break_force"] == pytest.approx(100.0)
    assert capture_metrics["controls"]["break_force_log10"] == pytest.approx(2.0)
    assert capture_metrics["metrics"]["break_force_log10"] == pytest.approx(2.0)


def test_rigid_distance_spring_panel_edits_public_spring_parameters() -> None:
    _require_simulation_symbols("World")

    setup = rigid_distance_spring.build()
    controller = setup.info["rigid_distance_spring_controller"]

    builder = _ScriptedPanelBuilder(
        slider_values={
            "Rest length": 0.62,
            "Soft stiffness": 72.0,
            "Stiff stiffness": 280.0,
            "Offset stiffness": 160.0,
        },
    )
    setup.panels[0].build(builder, object())

    assert "slider:Rest length:0.25:0.75" in builder.events
    assert "slider:Soft stiffness:10.0:100.0" in builder.events
    assert "slider:Stiff stiffness:80.0:320.0" in builder.events
    assert "slider:Offset stiffness:40.0:220.0" in builder.events
    assert controller.rest_length == pytest.approx(0.62)
    assert controller.soft_stiffness == pytest.approx(72.0)
    assert controller.stiff_stiffness == pytest.approx(280.0)
    assert controller.offset_stiffness == pytest.approx(160.0)
    assert controller.world.get_rigid_body_distance_spring_parameters(
        "soft_distance_spring"
    ) == pytest.approx((0.62, 72.0))
    assert controller.world.get_rigid_body_distance_spring_parameters(
        "stiff_distance_spring"
    ) == pytest.approx((0.62, 280.0))
    assert controller.world.get_rigid_body_distance_spring_parameters(
        "offset_distance_spring"
    ) == pytest.approx((0.62, 160.0))
    capture_metrics = setup.info["capture_metrics"]()
    assert capture_metrics["controls"]["rest_length"] == pytest.approx(0.62)
    assert capture_metrics["controls"]["soft_stiffness"] == pytest.approx(72.0)
    assert capture_metrics["controls"]["stiff_stiffness"] == pytest.approx(280.0)
    assert capture_metrics["controls"]["offset_stiffness"] == pytest.approx(160.0)


def test_rigid_joint_motor_limits_panel_edits_public_limits() -> None:
    _require_simulation_symbols("World", "JointSpec")

    setup = rigid_joint_motor_limits.build()
    controller = setup.info["rigid_joint_motor_limit_controller"]

    builder = _ScriptedPanelBuilder(
        slider_values={
            "Command speed": 0.73,
            "Velocity limit": 0.22,
            "Position upper limit": 0.47,
            "Requested force": 17.0,
            "Effort cap": 4.25,
        },
    )
    setup.panels[0].build(builder, object())

    assert "slider:Command speed:0.05:0.85" in builder.events
    assert "slider:Velocity limit:0.05:0.5" in builder.events
    assert "slider:Position upper limit:0.12:0.65" in builder.events
    assert "slider:Requested force:2.0:22.0" in builder.events
    assert "slider:Effort cap:1.0:9.0" in builder.events
    assert controller.command_speed == pytest.approx(0.73)
    assert controller.velocity_limit == pytest.approx(0.22)
    assert controller.position_limit == pytest.approx(0.47)
    assert controller.force_command == pytest.approx(17.0)
    assert controller.effort_limit == pytest.approx(4.25)
    assert controller.motor_joint.command_velocity.tolist() == pytest.approx([0.73])
    assert controller.motor_joint.velocity_lower_limits.tolist() == pytest.approx(
        [-0.22]
    )
    assert controller.motor_joint.velocity_upper_limits.tolist() == pytest.approx(
        [0.22]
    )
    assert controller.limit_joint.position_lower_limits.tolist() == pytest.approx(
        [-0.25]
    )
    assert controller.limit_joint.position_upper_limits.tolist() == pytest.approx(
        [0.47]
    )
    assert controller.limited_force_joint.force.tolist() == pytest.approx([17.0])
    assert controller.limited_force_joint.effort_lower_limits.tolist() == pytest.approx(
        [-4.25]
    )
    assert controller.limited_force_joint.effort_upper_limits.tolist() == pytest.approx(
        [4.25]
    )
    assert controller.open_force_joint.force.tolist() == pytest.approx([17.0])
    assert controller.open_force_joint.effort_upper_limits.tolist() == pytest.approx(
        [17.0]
    )
    capture_metrics = setup.info["capture_metrics"]()
    assert capture_metrics["controls"]["command_speed"] == pytest.approx(0.73)
    assert capture_metrics["controls"]["velocity_limit"] == pytest.approx(0.22)
    assert capture_metrics["controls"]["position_limit"] == pytest.approx(0.47)
    assert capture_metrics["controls"]["force_command"] == pytest.approx(17.0)
    assert capture_metrics["controls"]["effort_limit"] == pytest.approx(4.25)


def test_rigid_multibody_solver_family_panel_edits_execution_controls() -> None:
    _require_simulation_symbols("World", "LoopClosureSpec", "MultibodyOptions")

    setup = rigid_multibody_solver_family.build()
    controller = setup.info["rigid_multibody_solver_family_controller"]
    assert setup.pre_step is not None
    setup.pre_step()
    assert controller.primary_world.time > 0.0

    target_executor = len(controller._executors) - 1
    builder = _ScriptedPanelBuilder(
        select_values={"Executor": target_executor},
        slider_values={"Gravity scale": 0.45},
    )
    setup.panels[0].build(builder, object())

    assert any(event.startswith("select:Executor:") for event in builder.events)
    assert "slider:Gravity scale:0.0:1.8" in builder.events
    assert controller.executor_index == target_executor
    assert controller.gravity_scale == pytest.approx(0.45)
    assert controller.primary_world.time == pytest.approx(0.0)
    for case in controller.cases:
        assert case.world.time == pytest.approx(0.0)
        assert case.world.gravity[2] == pytest.approx(-9.81 * 0.45)
    capture_metrics = setup.info["capture_metrics"]()
    assert capture_metrics["executor"] == controller._executor_label()
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        target_executor
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(0.45)


def test_rigid_loop_closure_panel_edits_execution_controls() -> None:
    _require_simulation_symbols(
        "World",
        "LoopClosureSpec",
        "LoopClosureFamily",
        "ClosureDynamicsPolicy",
        "MultibodyOptions",
    )

    setup = rigid_loop_closure.build()
    controller = setup.info["rigid_loop_closure_controller"]
    assert setup.pre_step is not None
    setup.pre_step()
    assert controller.primary_world.time > 0.0

    if len(controller._executors) > 1:
        executor_only_builder = _ScriptedPanelBuilder(
            select_values={"Executor": 1},
        )
        setup.panels[0].build(executor_only_builder, object())

        assert controller.executor_index == 1
        assert controller.primary_world.time == pytest.approx(0.0)
        for case in controller.cases:
            assert case.world.time == pytest.approx(0.0)
        assert f"text:executor: {controller._executors[1][0]}" in (
            executor_only_builder.events
        )
        setup.pre_step()
        assert controller.primary_world.time > 0.0

    target_executor = len(controller._executors) - 1
    builder = _ScriptedPanelBuilder(
        select_values={"Executor": target_executor},
        slider_values={"Gravity scale": 0.55},
    )
    setup.panels[0].build(builder, object())

    assert any(event.startswith("select:Executor:") for event in builder.events)
    assert "slider:Gravity scale:0.0:1.4" in builder.events
    assert (
        f"text:executor: {controller._executors[target_executor][0]}"
        in builder.events
    )
    assert controller.executor_index == target_executor
    assert controller.gravity_scale == pytest.approx(0.55)
    assert controller.primary_world.time == pytest.approx(0.0)
    for case in controller.cases:
        assert case.world.time == pytest.approx(0.0)
        assert case.world.gravity[2] == pytest.approx(-9.81 * 0.55)
    capture_metrics = setup.info["capture_metrics"]()
    assert capture_metrics["executor"] == controller._executors[target_executor][0]
    assert capture_metrics["controls"]["executor_index"] == pytest.approx(
        target_executor
    )
    assert capture_metrics["controls"]["gravity_scale"] == pytest.approx(0.55)


def test_rigid_joint_passive_parameters_panel_edits_drive_forces() -> None:
    _require_simulation_symbols("World", "JointSpec")

    setup = rigid_joint_passive_parameters.build()
    controller = setup.info["rigid_joint_passive_parameters_controller"]

    builder = _ScriptedPanelBuilder(
        slider_values={
            "Hold force": 8.0,
            "Armature drive force": 12.0,
        },
    )
    setup.panels[0].build(builder, object())

    assert "slider:Hold force:0.0:12.0" in builder.events
    assert "slider:Armature drive force:0.0:18.0" in builder.events
    assert controller.hold_force == pytest.approx(8.0)
    assert controller.armature_force == pytest.approx(12.0)
    stiction = next(lane for lane in controller.lanes if lane.key == "stiction")
    armature_reference = next(
        lane for lane in controller.lanes if lane.key == "armature_reference"
    )
    armature_heavy = next(
        lane for lane in controller.lanes if lane.key == "armature_heavy"
    )
    assert stiction.joint.force.tolist() == pytest.approx([8.0])
    assert armature_reference.joint.force.tolist() == pytest.approx([12.0])
    assert armature_heavy.joint.force.tolist() == pytest.approx([12.0])
    capture_metrics = setup.info["capture_metrics"]()
    assert capture_metrics["controls"]["hold_force"] == pytest.approx(8.0)
    assert capture_metrics["controls"]["armature_force"] == pytest.approx(12.0)
    assert capture_metrics["lanes"]["stiction"]["metrics"]["force"] == pytest.approx(
        8.0
    )
    assert capture_metrics["lanes"]["stiction"]["metrics"][
        "status"
    ] == "force exceeds friction"
    assert capture_metrics["lanes"]["slip"]["metrics"]["status"] == (
        "force exceeds friction"
    )
    assert capture_metrics["lanes"]["armature_heavy"]["metrics"][
        "force"
    ] == pytest.approx(12.0)


def test_robot_puppet_world_scenes_expose_pose_panels() -> None:
    _require_simulation_symbols("World", "add_skeleton", "ReadOptions")

    for scene, expected_title in (
        (robot_puppets.ATLAS_PUPPET, "Atlas Puppet"),
        (robot_puppets.HUBO_PUPPET, "Hubo Puppet"),
    ):
        setup = scene.build()
        builder = _FakePanelBuilder()

        assert setup.force_drag is not None
        assert [panel.title for panel in setup.panels] == [expected_title]
        assert setup.info["dofs"] > 0
        assert setup.info["visual_links"] > 0

        setup.panels[0].build(builder, object())

        assert "slider:Pose blend:0.0:1.0" in builder.events
        assert "button:Reach pose" in builder.events
        assert "button:Crouch pose" in builder.events
        assert "button:Neutral pose" in builder.events
        assert any(event.startswith("plot:Root height:") for event in builder.events)
        assert "text:External force" in builder.events


def test_g1_puppet_stays_asset_gated_placeholder() -> None:
    setup = robot_puppets.G1_PUPPET.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["G1 Puppet"]
    assert setup.info["planned_world_port"] == "g1_puppet"

    setup.panels[0].build(builder, object())

    assert "text:status: planned World demo" in builder.events
    assert "text:legacy seed: g1_puppet" in builder.events
    assert any(event.startswith("text:target: ") for event in builder.events)


def test_planned_world_port_placeholders_expose_status_panels() -> None:
    for scene in [
        planned.INVERSE_KINEMATICS,
        planned.SIMBICON_WALKING,
        planned.OPERATIONAL_SPACE_CONTROL,
        planned.ROBOT_PUPPETS,
        planned.MOBILE_MANIPULATION,
    ]:
        setup = scene.build()
        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [scene.title]
        assert setup.info["planned_world_port"] == scene.id

        setup.panels[0].build(builder, object())

        assert "text:status: planned World demo" in builder.events
        assert any(event.startswith("text:legacy seeds: ") for event in builder.events)
        assert any(event.startswith("text:target: ") for event in builder.events)


def test_plan083_cpu_corpus_placeholders_expose_status_panels() -> None:
    runtime_scene_ids = set()

    for scene in plan083_unified_newton_barrier.PLAN083_SCENES:
        setup = scene.build()
        if setup.info.get("runtime_smoke_scene"):
            runtime_scene_ids.add(scene.id)
            continue

        builder = _FakePanelBuilder()

        assert [panel.title for panel in setup.panels] == [scene.title]
        assert setup.info["plan083_cpu_corpus_scene"] == scene.id
        assert setup.info["plan083_row_ids"]

        setup.panels[0].build(builder, object())

        assert "text:status: planned PLAN-083 CPU corpus scene" in builder.events
        assert any(event.startswith("text:rows: ") for event in builder.events)
        assert any(event.startswith("text:smoke: pixi run py-demos") for event in builder.events)
        assert any(
            event.startswith("text:visual: pixi run py-demo-capture")
            for event in builder.events
        )
        assert any(event.startswith("text:benchmark: ") for event in builder.events)
        assert any(event.startswith("text:limitation: ") for event in builder.events)

    assert runtime_scene_ids == {
        "plan083_candy",
        "plan083_hanging_bridge",
        "plan083_lying_flat",
        "plan083_nunchaku",
        "plan083_precession",
        "plan083_pulley_system",
        "plan083_ragdolls",
        "plan083_terrain_vehicle",
        "plan083_umbrella",
        "plan083_windmill",
    }


def test_plan083_hanging_bridge_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_hanging_bridge"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Hanging Bridge"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:point connections: 4" in builder.events
    assert (
        "text:benchmark: pixi run bm-plan083-cpu-hanging-bridge-packet"
        in builder.events
    )
    assert any(event.startswith("text:world time: ") for event in builder.events)


def test_plan083_nunchaku_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_nunchaku"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Nunchaku"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:revolute joints: 1" in builder.events
    assert "text:benchmark: pixi run bm-plan083-cpu-nunchaku-packet" in builder.events
    assert any(event.startswith("text:swinging tip radius: ") for event in builder.events)


def test_plan083_windmill_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_windmill"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Windmill"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:revolute joints: 1" in builder.events
    assert "text:benchmark: pixi run bm-plan083-cpu-windmill-packet" in builder.events
    assert any(event.startswith("text:blade tip radius: ") for event in builder.events)


def test_plan083_umbrella_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_umbrella"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Umbrella"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:revolute joints: 1" in builder.events
    assert "text:point connections: 2" in builder.events
    assert "text:benchmark: pixi run bm-plan083-cpu-umbrella-packet" in builder.events
    assert any(event.startswith("text:canopy span: ") for event in builder.events)


def test_plan083_candy_exposes_runtime_status_panel() -> None:
    _require_simulation_symbols("DeformableBodyOptions", "World")
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_candy"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Candy"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["deformable_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: deformable IPC World.step" in builder.events
    assert "text:deformable bodies: 1" in builder.events
    assert "text:nodes: 25" in builder.events
    assert "text:surface triangles: 32" in builder.events
    assert "text:benchmark: pixi run bm-plan083-cpu-candy-packet" in builder.events
    assert any(event.startswith("text:mean cloth height: ") for event in builder.events)


def test_plan083_lying_flat_exposes_runtime_status_panel() -> None:
    _require_simulation_symbols("DeformableBodyOptions", "World")
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_lying_flat"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Lying Flat"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["deformable_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: deformable IPC World.step" in builder.events
    assert "text:rigid obstacles: 4" in builder.events
    assert "text:deformable bodies: 1" in builder.events
    assert "text:nodes: 24" in builder.events
    assert "text:surface triangles: 30" in builder.events
    assert (
        "text:benchmark: pixi run bm-plan083-cpu-lying-flat-packet"
        in builder.events
    )
    assert any(event.startswith("text:mean cloth height: ") for event in builder.events)


def test_plan083_terrain_vehicle_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_terrain_vehicle"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Terrain Vehicle"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:passive wheels: 4" in builder.events
    assert "text:revolute joints: 4" in builder.events
    assert (
        "text:benchmark: pixi run bm-plan083-cpu-terrain-vehicle-packet"
        in builder.events
    )
    assert any(event.startswith("text:chassis height: ") for event in builder.events)
    assert any(
        event.startswith("text:min wheel clearance: ") for event in builder.events
    )


def test_plan083_precession_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_precession"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Precession"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:rolling wheels: 1" in builder.events
    assert "text:benchmark: pixi run bm-plan083-cpu-precession-packet" in builder.events
    assert any(
        event.startswith("text:wheel ground clearance: ") for event in builder.events
    )
    assert any(event.startswith("text:spin rate: ") for event in builder.events)


def test_plan083_ragdolls_exposes_runtime_status_panel() -> None:
    scene = next(
        scene
        for scene in plan083_unified_newton_barrier.PLAN083_SCENES
        if scene.id == "plan083_ragdolls"
    )
    setup = scene.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["PLAN-083 Ragdolls"]
    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["rigid_body_solver"] == "ipc"

    setup.panels[0].build(builder, object())

    assert "text:status: reduced runtime smoke scene" in builder.events
    assert "text:solver: rigid IPC World.step" in builder.events
    assert "text:reduced ragdoll bodies: 6" in builder.events
    assert "text:revolute joints: 5" in builder.events
    assert "text:benchmark: pixi run bm-plan083-cpu-ragdoll-packet" in builder.events
    assert any(event.startswith("text:min leg clearance: ") for event in builder.events)


def test_ipc_deformable_scene_exposes_diagnostics_panel() -> None:
    _require_simulation_symbols("DeformableBodyOptions", "DeformableEdge", "World")

    setup = ipc_deformable_friction_slide.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["IPC Friction Slide"]

    setup.panels[0].build(builder, object())

    assert any(event.startswith("plot:Min z:") for event in builder.events)
    assert any(
        event.startswith("text:solver: deformable IPC") for event in builder.events
    )


def test_diff_drone_scene_exposes_replay_panel() -> None:
    _require_simulation_symbols("World")

    setup = diff_drone_liftoff.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["Diff Drone Lift-Off"]

    setup.panels[0].build(builder, object())

    assert "slider:Playback stride:1.0:8.0" in builder.events
    assert "button:Reset replay" in builder.events
    assert any(event.startswith("plot:Aware height:") for event in builder.events)
    assert any(event.startswith("plot:Aware thrust:") for event in builder.events)
    assert any(event.startswith("plot:Aware loss:") for event in builder.events)
    assert any(
        event.startswith("plot:Aware thrust gradient:") for event in builder.events
    )
    assert any(event.startswith("plot:Analytic loss:") for event in builder.events)
    assert setup.info["gradient_modes"] == ["ANALYTIC", "COMPLEMENTARITY_AWARE"]
    assert setup.info["aware_history_count"] >= 1
    assert setup.info["naive_history_count"] >= 1


def test_diff_pre_contact_surrogate_scene_exposes_diagnostic_panel() -> None:
    _require_simulation_symbols("World")

    setup = diff_pre_contact_surrogate.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == [
        "Diff Pre-Contact Surrogate"
    ]

    setup.panels[0].build(builder, object())

    assert any(event.startswith("text:forward: identical") for event in builder.events)
    assert any(event.startswith("text:forward max diff:") for event in builder.events)
    assert any(
        event.startswith("text:ANALYTIC freefall error:")
        for event in builder.events
    )
    assert any(event.startswith("text:SURROGATE block:") for event in builder.events)
    assert any(event.startswith("plot:vertical sensitivity:") for event in builder.events)
    assert "button:Toggle derivative details" in builder.events
    assert setup.info["gradient_modes"] == ["ANALYTIC", "PRE_CONTACT_SURROGATE"]
    assert setup.info["initial_clearance"] == pytest.approx(0.5)


def test_diff_trajectory_scenes_expose_replay_panels() -> None:
    _require_simulation_symbols("World")

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


def test_replay_scrubber_exposes_timeline_panel() -> None:
    _require_simulation_symbols("World")

    setup = replay_scrubber.build()
    builder = _FakePanelBuilder()

    assert [panel.title for panel in setup.panels] == ["Replay Timeline"]
    assert setup.panels[0].dock_side == "bottom"
    assert setup.panels[0].initial_size == (960.0, 320.0)
    assert setup.info["timeline_frame_count"] == setup.info["recorded_frames"]
    assert setup.info["timeline_duration"] > 0.0

    setup.panels[0].build(builder, object())

    assert any(
        event.startswith("timeline:Timeline##replay_timeline:0.0:")
        for event in builder.events
    )
    assert any(event.endswith(":181:181:181:Ball height") for event in builder.events)
    assert "checkbox:Loop playback" in builder.events
    assert any(event.startswith("select:Rate:") for event in builder.events)
    assert "collapsing:Cursor details:False" in builder.events
    assert "table:Cursor details table:Track,Value" in builder.events
    assert "button:-10" in builder.events
    assert "button:+10" in builder.events


def test_ipc_fem_buckle_scene_exposes_compression_panel() -> None:
    _require_simulation_symbols(
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
    _require_simulation_symbols(
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
    _require_simulation_symbols("DeformableBodyOptions", "DeformableEdge", "World")

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
    _require_simulation_symbols("DeformableBodyOptions", "DeformableEdge", "World")

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
    _require_simulation_symbols(
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
    _require_simulation_symbols(
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
    _require_simulation_symbols(
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
    _require_simulation_symbols(
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


@pytest.mark.skipif(
    not hasattr(dart, "gui") or not hasattr(dart.gui, "describe_shape"),
    reason="GUI descriptor extraction is not available in this build",
)
def test_world_bridge_force_drag_uses_renderable_id_and_restores_rigid_force() -> None:
    sx_world = _FakeWorld()
    target = _FakeRigidBody()
    other = _FakeRigidBody()
    other.name = "other"

    bridge = WorldRenderBridge(sx_world, name="force_drag_test")
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
        name
        for name, physics_object in bridge._by_name.items()
        if physics_object is target
    )
    target_id = next(
        int(renderable.id)
        for renderable in bridge.renderable_provider()
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
    not hasattr(dart, "gui") or not hasattr(dart.gui, "describe_shape"),
    reason="GUI descriptor extraction is not available in this build",
)
def test_world_bridge_external_force_panel_reports_disabled_and_static_targets() -> (
    None
):
    sx_world = _FakeWorld()
    static_target = _FakeRigidBody()
    static_target.name = "ground"
    static_target.is_static = True

    bridge = WorldRenderBridge(sx_world, name="external_force_state_test")
    bridge.add_rigid_body_visual(
        static_target,
        dart.BoxShape(np.array([0.1, 0.1, 0.1])),
        (0.5, 0.5, 0.5),
        name="static_visual",
    )
    static_id = next(
        int(renderable.id)
        for renderable in bridge.renderable_provider()
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
    dynamic_bridge = WorldRenderBridge(_FakeWorld(), name="external_force_hint_test")
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
