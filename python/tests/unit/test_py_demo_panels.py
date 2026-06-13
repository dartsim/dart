from __future__ import annotations

import dartpy as dart
import numpy as np
import pytest
from examples.demos._world_bridge import WorldRenderBridge
from examples.demos.registry import make_demo_scenes
from examples.demos.runner import (
    DEFAULT_INITIAL_SCENE_ID,
    DEFAULT_SCENE_BUILD_TIMEOUT_MS,
    PythonDemoScene,
    ScenePanel,
    SceneSetup,
    _attach_replay_controls,
    _default_initial_scene_args,
    _has_world_replay_api,
    _make_world_factory,
    _scene_build_timeout_ms,
    _validate_scene,
)

from examples.demos.scenes import (
    articulated,
    atlas_simbicon,
    contact,
    diff_cartpole_trajopt,
    diff_drone_liftoff,
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
    rigid_fixed_joint,
    rigid_ipc,
    rigid_ipc_incline,
    rigid_ipc_pile,
    rigid_ipc_tunnel,
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
    assert result == (None, None, [panel], None, None)


def test_make_world_factory_returns_debug_provider() -> None:
    def debug_provider() -> object:
        return object()

    scene = PythonDemoScene(
        id="debug_provider_scene",
        title="Debug Provider Scene",
        category="Tests",
        summary="Has a custom debug provider.",
        build=lambda: SceneSetup(world=object(), debug_provider=debug_provider),
    )

    result = _make_world_factory(scene)()

    assert result == (None, None, None, None, debug_provider)


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
    pre_step, force_drag, panels, renderable_provider, debug_provider = result
    assert renderable_provider is None
    assert debug_provider is None
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
        "planned_collision_sandbox",
        "planned_mobile_manipulation",
        "gui_fidelity_debug_visuals",
        "plan083_abd_complex_geometry",
        "plan083_abd_fem_coupling",
        "diff_throw_to_target",
        "diff_cartpole_trajopt",
        "diff_drone_liftoff",
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


def test_default_py_demos_launch_uses_replay_timeline_without_reordering() -> None:
    assert DEFAULT_INITIAL_SCENE_ID == "replay_scrubber"
    assert _default_initial_scene_args([], None, {}) == [
        "--scene",
        "replay_scrubber",
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

    def set_paused(self, paused: bool) -> None:
        self.paused = bool(paused)

    def request_single_step(self) -> None:
        self.single_step_requests += 1


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
        self.events.append(
            f"timeline:{label}:{minimum}:{maximum}:"
            f"{len(value_track)}:{len(marker_track)}:{len(cursor_track)}:"
            f"{value_track_label}"
        )
        return False, value

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        self.events.append(f"checkbox:{label}")
        return False, value

    def select(
        self, label: str, selected_index: int, choices: list[str]
    ) -> tuple[bool, int]:
        self.events.append(f"select:{label}:{selected_index}:{','.join(choices)}")
        return False, selected_index

    def button(self, label: str) -> bool:
        self.events.append(f"button:{label}")
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
        checkbox_values: dict[str, bool] | None = None,
        timeline_values: dict[str, float] | None = None,
    ) -> None:
        super().__init__()
        self.clicked_buttons = clicked_buttons or set()
        self.checkbox_values = checkbox_values or {}
        self.timeline_values = timeline_values or {}

    def button(self, label: str) -> bool:
        self.events.append(f"button:{label}")
        return label in self.clicked_buttons

    def checkbox(self, label: str, value: bool) -> tuple[bool, bool]:
        self.events.append(f"checkbox:{label}")
        if label in self.checkbox_values:
            return True, self.checkbox_values[label]
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
        self.events.append(
            f"timeline:{label}:{minimum}:{maximum}:"
            f"{len(value_track)}:{len(marker_track)}:{len(cursor_track)}:"
            f"{value_track_label}"
        )
        if label in self.timeline_values:
            return True, self.timeline_values[label]
        return False, value


def test_high_value_world_scenes_expose_custom_panels() -> None:
    sx = _require_simulation_symbols("World")

    cases = [
        (articulated, "Articulated"),
        (floating_base, "Floating Base"),
        (contact, "Contact"),
        (rigid_body, "Rigid Bodies"),
        (rigid_ipc, "Rigid IPC Contact"),
        (rigid_ipc_incline, "Rigid IPC Incline"),
        (rigid_ipc_pile, "Rigid IPC Pile"),
        (rigid_ipc_tunnel, "Rigid IPC Tunnel"),
        (atlas_simbicon, "Atlas SIMBICON"),
        (variational_chain, "Variational Chain"),
        (variational_tumbler, "Variational Tumbler"),
    ]
    if hasattr(sx.World(), "add_rigid_body_fixed_joint"):
        cases.insert(4, (rigid_fixed_joint, "Rigid Fixed Joint"))

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
        planned.COLLISION_SANDBOX,
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
