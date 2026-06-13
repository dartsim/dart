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
    lcp_physics,
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
        "plan083_abd_complex_geometry",
        "plan083_abd_fem_coupling",
        "diff_throw_to_target",
        "diff_cartpole_trajopt",
        "diff_drone_liftoff",
        "lcp_physics",
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
        (lcp_physics, "LCP Physics"),
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


def _write_lcp_profile_evidence(
    evidence_path, columns: tuple[str, ...] | None = None, **overrides: str
) -> None:
    row = {
        "category": "Standard",
        "solver": "Dantzig",
        "problem_size": "12",
        "lcp_dimension": "12",
        "contact_count": "",
        "solver_identity_schema_version": "1",
        "solver_manifest_index": "1",
        "solver_family_pivoting": "1",
        "solver_family_projection": "0",
        "solver_family_newton": "0",
        "solver_family_other": "0",
        "time_ns": "1",
        "contract_ok": "1",
        "iterations": "0",
        "residual": "0",
        "complementarity": "0",
        "bound_violation": "0",
        "solver_supports_standard": "1",
        "solver_supports_boxed": "1",
        "solver_supports_friction_index": "1",
        "solver_supports_problem": "1",
        "problem_type_standard": "1",
        "problem_type_boxed": "0",
        "problem_type_friction_index": "0",
        "problem_type_invalid": "0",
    }
    row.update(overrides)
    output_columns = (
        columns or lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS
    )
    evidence_path.write_text(
        ",".join(output_columns)
        + "\n"
        + ",".join(row[column] for column in output_columns)
        + "\n",
        encoding="utf-8",
    )


@pytest.mark.parametrize(
    (
        "solver",
        "solver_manifest_index",
        "solver_supports_boxed",
        "solver_supports_friction_index",
        "solver_supports_problem",
        "expected_error",
    ),
    [
        (
            "Lemke",
            "2",
            "0",
            "0",
            "0",
            "non-native LCP performance profile evidence row: Boxed/Lemke",
        ),
        (
            "Dantzig",
            "1",
            "1",
            "1",
            "0",
            "unsupported LCP performance profile evidence row: Boxed/Dantzig",
        ),
    ],
)
def test_lcp_physics_profile_summary_rejects_non_native_evidence_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    solver: str,
    solver_manifest_index: str,
    solver_supports_boxed: str,
    solver_supports_friction_index: str,
    solver_supports_problem: str,
    expected_error: str,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(
        evidence_path,
        category="Boxed",
        solver=solver,
        solver_manifest_index=solver_manifest_index,
        solver_supports_boxed=solver_supports_boxed,
        solver_supports_friction_index=solver_supports_friction_index,
        solver_supports_problem=solver_supports_problem,
        problem_type_standard="0",
        problem_type_boxed="1",
    )
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(RuntimeError, match=expected_error):
        lcp_physics._performance_profile_evidence_summary_rows()


@pytest.mark.parametrize(
    ("category", "problem_type_counters", "expected_error"),
    [
        (
            "Unknown",
            "0,1,0,0",
            "unknown LCP performance profile evidence category: 'Unknown'",
        ),
        (
            "Boxed",
            "1,0,0,0",
            "mismatched LCP performance profile evidence row: "
            "Boxed/Dantzig has problem_type_standard=1",
        ),
        (
            "Boxed",
            "0,1,0,1",
            "mismatched LCP performance profile evidence row: "
            "Boxed/Dantzig has problem_type_invalid=1",
        ),
    ],
)
def test_lcp_physics_profile_summary_rejects_mismatched_problem_type_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    category: str,
    problem_type_counters: str,
    expected_error: str,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    (
        problem_type_standard,
        problem_type_boxed,
        problem_type_friction_index,
        problem_type_invalid,
    ) = problem_type_counters.split(",")
    _write_lcp_profile_evidence(
        evidence_path,
        category=category,
        problem_type_standard=problem_type_standard,
        problem_type_boxed=problem_type_boxed,
        problem_type_friction_index=problem_type_friction_index,
        problem_type_invalid=problem_type_invalid,
    )
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(RuntimeError, match=expected_error):
        lcp_physics._performance_profile_evidence_summary_rows()


@pytest.mark.parametrize(
    ("overrides", "expected_error"),
    [
        (
            {"solver_identity_schema_version": "2"},
            "Standard/Dantzig has solver_identity_schema_version=2; expected 1",
        ),
        (
            {"solver_identity_schema_version": "1.25"},
            "Standard/Dantzig has solver_identity_schema_version=None; expected 1",
        ),
        (
            {"solver_manifest_index": "2"},
            "Standard/Dantzig has solver_manifest_index=2; expected 1",
        ),
        (
            {"solver_manifest_index": "1.25"},
            "Standard/Dantzig has solver_manifest_index=None; expected 1",
        ),
        (
            {"solver_family_pivoting": "0"},
            "Standard/Dantzig has solver_family_pivoting=0; "
            "expected 1 for Pivoting",
        ),
        (
            {"solver_family_projection": "1"},
            "Standard/Dantzig has solver_family_projection=1; "
            "expected 0 for Pivoting",
        ),
        (
            {"solver": "MissingSolver"},
            "unknown LCP performance profile evidence solver: 'MissingSolver'",
        ),
    ],
)
def test_lcp_physics_profile_summary_rejects_stale_solver_identity_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    overrides: dict[str, str],
    expected_error: str,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(evidence_path, **overrides)
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(RuntimeError, match=expected_error):
        lcp_physics._performance_profile_evidence_summary_rows()


@pytest.mark.parametrize(
    ("overrides", "expected_error"),
    [
        (
            {"solver_supports_standard": "0"},
            "Standard/Dantzig has solver_supports_standard=0; expected 1",
        ),
        (
            {"solver_supports_boxed": "0"},
            "Standard/Dantzig has solver_supports_boxed=0; expected 1",
        ),
        (
            {"solver_supports_friction_index": "0"},
            "Standard/Dantzig has solver_supports_friction_index=0; "
            "expected 1",
        ),
    ],
)
def test_lcp_physics_profile_summary_rejects_stale_support_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    overrides: dict[str, str],
    expected_error: str,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(evidence_path, **overrides)
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(RuntimeError, match=expected_error):
        lcp_physics._performance_profile_evidence_summary_rows()


@pytest.mark.parametrize(
    ("overrides", "expected_error"),
    [
        (
            {"problem_size": "0"},
            "Standard/Dantzig has problem_size='0'",
        ),
        (
            {"problem_size": "12.5"},
            "Standard/Dantzig has problem_size='12.5'",
        ),
        (
            {"lcp_dimension": "11"},
            "Standard/Dantzig has lcp_dimension=11; expected 12",
        ),
        (
            {"contact_count": "-1"},
            "Standard/Dantzig has contact_count='-1'",
        ),
        (
            {
                "category": "FrictionIndex",
                "problem_size": "4",
                "lcp_dimension": "12",
                "contact_count": "3",
                "problem_type_standard": "0",
                "problem_type_friction_index": "1",
            },
            "FrictionIndex/Dantzig has contact_count=3; expected 4",
        ),
        (
            {"time_ns": "0"},
            "Standard/Dantzig has time_ns='0'",
        ),
        (
            {"contract_ok": "0"},
            "Standard/Dantzig has contract_ok='0'",
        ),
        (
            {"iterations": "-1"},
            "Standard/Dantzig has iterations='-1'",
        ),
        (
            {"iterations": "1.25"},
            "Standard/Dantzig has iterations='1.25'",
        ),
        (
            {"residual": "nan"},
            "Standard/Dantzig has residual='nan'",
        ),
        (
            {"complementarity": "-1"},
            "Standard/Dantzig has complementarity='-1'",
        ),
        (
            {"bound_violation": "inf"},
            "Standard/Dantzig has bound_violation='inf'",
        ),
    ],
)
def test_lcp_physics_profile_summary_rejects_invalid_numeric_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    overrides: dict[str, str],
    expected_error: str,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(evidence_path, **overrides)
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(RuntimeError, match=expected_error):
        lcp_physics._performance_profile_evidence_summary_rows()


def test_lcp_physics_profile_summary_rejects_missing_evidence_columns(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(
        evidence_path,
        columns=tuple(
            column
            for column in lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS
            if column != "time_ns"
        ),
    )
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(
        RuntimeError,
        match="missing required columns: \\['time_ns'\\]",
    ):
        lcp_physics._performance_profile_evidence_summary_rows()


def test_lcp_physics_profile_summary_rejects_duplicate_evidence_columns(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(
        evidence_path,
        columns=(
            *lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS,
            "time_ns",
        ),
    )
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(
        RuntimeError,
        match="contains duplicate columns: \\['time_ns'\\]",
    ):
        lcp_physics._performance_profile_evidence_summary_rows()


def test_lcp_physics_profile_summary_rejects_duplicate_evidence_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(evidence_path)
    lines = evidence_path.read_text(encoding="utf-8").splitlines()
    evidence_path.write_text(
        "\n".join((*lines, lines[1])) + "\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(
        RuntimeError,
        match="duplicate LCP performance profile evidence row: Standard/Dantzig/12",
    ):
        lcp_physics._performance_profile_evidence_summary_rows()


def test_lcp_physics_profile_summary_rejects_empty_evidence_file(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    evidence_path.write_text(
        ",".join(lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS)
        + "\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(
        RuntimeError,
        match="LCP performance profile evidence has no rows",
    ):
        lcp_physics._performance_profile_evidence_summary_rows()


def test_lcp_physics_profile_summary_rejects_missing_evidence_surfaces(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    evidence_path = tmp_path / "performance_profile_evidence.csv"
    _write_lcp_profile_evidence(evidence_path)
    monkeypatch.setattr(
        lcp_physics, "_PERFORMANCE_PROFILE_EVIDENCE_PATH", evidence_path
    )

    with pytest.raises(
        RuntimeError,
        match="missing surfaces: \\['Boxed', 'FrictionIndex'\\]",
    ):
        lcp_physics._performance_profile_evidence_summary_rows()


def test_lcp_physics_profile_evidence_schema_rows_cover_required_columns() -> None:
    documented_fields: list[str] = []
    for row in lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_SCHEMA_ROWS:
        fields = [field.strip() for field in row["fields"].split(",")]
        assert all(fields)
        assert row["meaning"]
        documented_fields.extend(fields)

    assert documented_fields == list(
        lcp_physics._PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS
    )
    assert len(documented_fields) == len(set(documented_fields))


def test_lcp_physics_exposes_solver_manifest_and_benchmark_metadata() -> None:
    _require_simulation_symbols("World", "ContactSolverMethod")

    setup = lcp_physics.build()
    info = setup.info
    summary = info["solver_manifest_summary"]
    solver_rows = info["solver_rows"]
    solver_by_name = {row["name"]: row for row in solver_rows}
    expected_solver_names = {
        solver.get_name()
        for solver in (
            dart.DantzigSolver(),
            dart.LemkeSolver(),
            dart.BaraffSolver(),
            dart.DirectSolver(),
            dart.PgsSolver(),
            dart.SymmetricPsorSolver(),
            dart.JacobiSolver(),
            dart.RedBlackGaussSeidelSolver(),
            dart.BlockedJacobiSolver(),
            dart.BgsSolver(),
            dart.NncgSolver(),
            dart.SubspaceMinimizationSolver(),
            dart.ApgdSolver(),
            dart.TgsSolver(),
            dart.MinimumMapNewtonSolver(),
            dart.FischerBurmeisterNewtonSolver(),
            dart.PenalizedFischerBurmeisterNewtonSolver(),
            dart.InteriorPointSolver(),
            dart.MprgpSolver(),
            dart.ShockPropagationSolver(),
            dart.StaggeringSolver(),
            dart.AdmmSolver(),
            dart.SapSolver(),
            dart.BoxedSemiSmoothNewtonSolver(),
        )
    }

    assert summary == {
        "solver_count": 24,
        "standard_count": 23,
        "boxed_count": 15,
        "findex_count": 16,
    }
    assert len(solver_rows) == summary["solver_count"]
    assert len(solver_by_name) == len(solver_rows)
    assert set(solver_by_name) == expected_solver_names
    assert solver_by_name["Dantzig"]["boxed"] is True
    assert solver_by_name["BoxedSemiSmoothNewton"]["findex"] is True
    assert solver_by_name["MPRGP"]["boxed"] is False
    assert info["standalone_lcp_solvers_exposed_in_dartpy"] is True
    assert len(info["standalone_solver_rows"]) == summary["solver_count"]
    smoke_by_name = {row["name"]: row for row in info["standalone_solver_rows"]}
    assert len(smoke_by_name) == len(info["standalone_solver_rows"])
    assert set(smoke_by_name) == expected_solver_names
    for name, manifest_row in solver_by_name.items():
        smoke_row = smoke_by_name[name]
        assert smoke_row["native_standard"] is manifest_row["standard"]
        assert smoke_row["native_boxed"] is manifest_row["boxed"]
        assert smoke_row["native_findex"] is manifest_row["findex"]
    assert max(row["solution_error"] for row in info["standalone_solver_rows"]) < 1e-4
    assert {row["status"] for row in info["standalone_solver_rows"]} <= {
        "Success",
        "MaxIterations",
    }
    expected_problem_counts = {
        "standard_spd": 24,
        "ill_conditioned_standard": 24,
        "near_singular_standard": 24,
        "boxed_active_bounds": 24,
        "mass_ratio_boxed": 24,
        "singular_degenerate_boxed": 24,
        "friction_index_contact": 24,
        "active_friction_index_contact": 24,
        "moderate_scale_standard": 24,
    }
    expected_native_problem_counts = {
        "standard_spd": 23,
        "ill_conditioned_standard": 23,
        "near_singular_standard": 22,
        "boxed_active_bounds": 15,
        "mass_ratio_boxed": 15,
        "singular_degenerate_boxed": 15,
        "friction_index_contact": 16,
        "active_friction_index_contact": 16,
        "moderate_scale_standard": 22,
    }
    expected_problem_types = {
        "standard_spd": "Standard",
        "ill_conditioned_standard": "Standard",
        "near_singular_standard": "Standard",
        "boxed_active_bounds": "Boxed",
        "mass_ratio_boxed": "Boxed",
        "singular_degenerate_boxed": "Boxed",
        "friction_index_contact": "FrictionIndex",
        "active_friction_index_contact": "FrictionIndex",
        "moderate_scale_standard": "Standard",
    }
    expected_problem_dimensions = {
        "standard_spd": 3,
        "ill_conditioned_standard": 3,
        "near_singular_standard": 4,
        "boxed_active_bounds": 3,
        "mass_ratio_boxed": 8,
        "singular_degenerate_boxed": 4,
        "friction_index_contact": 3,
        "active_friction_index_contact": 6,
        "moderate_scale_standard": 12,
    }
    expected_findex_row_counts = {
        "standard_spd": 0,
        "ill_conditioned_standard": 0,
        "near_singular_standard": 0,
        "boxed_active_bounds": 0,
        "mass_ratio_boxed": 0,
        "singular_degenerate_boxed": 0,
        "friction_index_contact": 2,
        "active_friction_index_contact": 4,
        "moderate_scale_standard": 0,
    }
    expected_findex_contact_counts = {
        "standard_spd": 0,
        "ill_conditioned_standard": 0,
        "near_singular_standard": 0,
        "boxed_active_bounds": 0,
        "mass_ratio_boxed": 0,
        "singular_degenerate_boxed": 0,
        "friction_index_contact": 1,
        "active_friction_index_contact": 2,
        "moderate_scale_standard": 0,
    }
    expected_case_tolerances = {
        "mass_ratio_boxed": 5e-4,
    }
    problem_rows = info["standalone_problem_rows"]
    problem_summary_rows = info["standalone_problem_summary_rows"]
    solver_profile_rows = info["standalone_solver_profile_rows"]
    parameter_rows = info["advanced_solver_parameter_rows"]
    problem_summary_by_case = {row["case"]: row for row in problem_summary_rows}
    solver_profile_by_name = {row["solver"]: row for row in solver_profile_rows}
    parameter_by_solver = {row["solver"]: row for row in parameter_rows}
    assert len(problem_rows) == sum(expected_problem_counts.values())
    assert set(problem_summary_by_case) == set(expected_problem_counts)
    assert len(solver_profile_rows) == summary["solver_count"]
    assert set(solver_profile_by_name) == expected_solver_names
    assert set(parameter_by_solver) == {
        "Admm",
        "Apgd",
        "Bgs",
        "BlockedJacobi",
        "BoxedSemiSmoothNewton",
        "FischerBurmeisterNewton",
        "InteriorPoint",
        "Jacobi",
        "MinimumMapNewton",
        "Mprgp",
        "Nncg",
        "PenalizedFischerBurmeisterNewton",
        "Pgs",
        "RedBlackGaussSeidel",
        "Sap",
        "ShockPropagation",
        "SubspaceMinimization",
        "SymmetricPsor",
        "Tgs",
    }
    assert "epsilon_for_division" in parameter_by_solver["Pgs"]["parameters"]
    assert "adaptive_restart" in parameter_by_solver["Apgd"]["parameters"]
    assert "lambda_" in parameter_by_solver["PenalizedFischerBurmeisterNewton"][
        "parameters"
    ]
    assert "sigma" in parameter_by_solver["InteriorPoint"]["parameters"]
    assert "check_positive_definite" in parameter_by_solver["Mprgp"]["parameters"]
    assert "rho_init" in parameter_by_solver["Admm"]["parameters"]
    assert "regularization" in parameter_by_solver["Sap"]["parameters"]
    assert "max_line_search_steps" in parameter_by_solver["BoxedSemiSmoothNewton"][
        "parameters"
    ]
    assert "max_pgs_warm_start_iterations" in parameter_by_solver[
        "BoxedSemiSmoothNewton"
    ]["parameters"]
    assert "max_friction_index_exact_solve_dimension" in parameter_by_solver[
        "BoxedSemiSmoothNewton"
    ]["parameters"]
    assert parameter_by_solver["Pgs"]["benchmark_filter"] == "BM_LcpPgsRelaxationSweep"
    assert parameter_by_solver["Apgd"]["benchmark_filter"] == (
        "BM_LcpApgdRestartSweep"
    )
    assert parameter_by_solver["InteriorPoint"]["benchmark_filter"] == (
        "BM_LcpInteriorPointPathSweep"
    )
    assert parameter_by_solver["Mprgp"]["benchmark_filter"] == (
        "BM_LcpMprgpSpdCheckSweep"
    )
    assert parameter_by_solver["Admm"]["benchmark_filter"] == "BM_LcpAdmmRhoSweep"
    assert parameter_by_solver["Sap"]["benchmark_filter"] == (
        "BM_LcpSapRegularizationSweep"
    )
    assert parameter_by_solver["BoxedSemiSmoothNewton"]["benchmark_filter"] == (
        "BM_LcpBoxedSemiSmoothNewtonLineSearchSweep"
    )
    assert {row["surface"] for row in problem_summary_rows} == {
        "standard",
        "boxed",
        "findex",
    }
    assert all(row["contract_ok"] for row in problem_rows)
    assert {row["status"] for row in problem_rows} <= {"Success", "MaxIterations"}
    for case_name, expected_count in expected_problem_counts.items():
        summary_row = problem_summary_by_case[case_name]
        assert summary_row["solver_count"] == expected_count
        assert summary_row["problem_type"] == expected_problem_types[case_name]
        assert summary_row["lcp_dimension"] == expected_problem_dimensions[case_name]
        assert (
            summary_row["findex_row_count"]
            == expected_findex_row_counts[case_name]
        )
        assert (
            summary_row["findex_contact_count"]
            == expected_findex_contact_counts[case_name]
        )
        assert {
            row["problem_type"] for row in problem_rows if row["case"] == case_name
        } == {expected_problem_types[case_name]}
        assert {
            row["lcp_dimension"] for row in problem_rows if row["case"] == case_name
        } == {expected_problem_dimensions[case_name]}
        assert {
            row["findex_row_count"]
            for row in problem_rows
            if row["case"] == case_name
        } == {expected_findex_row_counts[case_name]}
        assert {
            row["findex_contact_count"]
            for row in problem_rows
            if row["case"] == case_name
        } == {expected_findex_contact_counts[case_name]}
        assert summary_row["native_solver_count"] == expected_native_problem_counts[
            case_name
        ]
        assert (
            summary_row["delegated_solver_count"]
            == expected_count - expected_native_problem_counts[case_name]
        )
        assert summary_row["contract_ok_count"] == expected_count
        assert summary_row["native_contract_ok_count"] == expected_native_problem_counts[
            case_name
        ]
        assert summary_row["delegated_contract_ok_count"] == (
            expected_count - expected_native_problem_counts[case_name]
        )
        assert summary_row["max_solution_error"] <= expected_case_tolerances.get(
            case_name, 2e-4
        )
        assert summary_row["max_residual"] <= 1e-3
        assert summary_row["max_complementarity"] <= 1e-3
        assert summary_row["fastest_solver"] in solver_by_name
        assert summary_row["fastest_elapsed_us"] >= 0.0
        assert summary_row["fastest_native_solver"] in solver_by_name
        assert summary_row["fastest_native_elapsed_us"] >= 0.0
    for solver_name, profile_row in solver_profile_by_name.items():
        manifest_row = solver_by_name[solver_name]
        expected_native_case_count = (
            4 * int(manifest_row["standard"])
            + 3 * int(manifest_row["boxed"])
            + 2 * int(manifest_row["findex"])
        )
        if solver_name == "Direct":
            expected_native_case_count = 2
        assert profile_row["problem_count"] == len(expected_problem_counts)
        assert profile_row["native_case_count"] == expected_native_case_count
        assert (
            profile_row["delegated_case_count"]
            == len(expected_problem_counts) - expected_native_case_count
        )
        assert profile_row["contract_ok_count"] == len(expected_problem_counts)
        assert profile_row["native_contract_ok_count"] == expected_native_case_count
        assert (
            profile_row["delegated_contract_ok_count"]
            == len(expected_problem_counts) - expected_native_case_count
        )
        assert profile_row["max_solution_error"] <= max(
            expected_case_tolerances.values(), default=2e-4
        )
        assert profile_row["max_residual"] <= 1e-3
        assert profile_row["max_complementarity"] <= 1e-3
        assert profile_row["total_elapsed_us"] >= 0.0
        assert profile_row["slowest_case"] in expected_problem_counts
    assert solver_profile_by_name["Dantzig"]["native_surfaces"] == (
        "standard 4/4, boxed 3/3, findex 2/2"
    )
    assert solver_profile_by_name["MPRGP"]["native_surfaces"] == "standard 4/4"
    assert solver_profile_by_name["Direct"]["native_surfaces"] == "standard 2/4"
    assert {
        row["solver"]
        for row in problem_rows
        if row["case"] == "near_singular_standard" and not row["native_supported"]
    } == {"Direct", "Staggering"}
    assert {
        row["solver"]
        for row in problem_rows
        if row["case"] == "moderate_scale_standard" and not row["native_supported"]
    } == {"Direct", "Staggering"}
    assert {
        row["solver"]
        for row in problem_rows
        if row["case"] == "boxed_active_bounds" and not row["native_supported"]
    } == {name for name, manifest_row in solver_by_name.items() if not manifest_row["boxed"]}
    assert {
        row["solver"]
        for row in problem_rows
        if row["case"] == "friction_index_contact" and not row["native_supported"]
    } == {name for name, manifest_row in solver_by_name.items() if not manifest_row["findex"]}
    assert info["benchmark_smoke_filter"] == "BM_LCP_COMPARE_SMOKE"
    assert "BM_LCP_COMPARE_SMOKE" in info["benchmark_command"]
    benchmark_by_packet = {
        row["packet"]: row for row in info["benchmark_packet_rows"]
    }
    assert {row["packet"] for row in info["live_packet_rows"]} == {
        "Sliding friction",
        "Static-friction ramp",
        "Billiard collision",
        "High-mass-ratio stack",
        "Thin card pile",
    }
    assert {row["packet"] for row in info["benchmark_packet_rows"]} >= {
        "active_set_transition",
        "active_set_scale",
        "active_friction_index_contact",
        "contact_solver_comparison_sweep",
        "contact_normal_standard_sweep",
        "singular_degenerate",
        "singular_degenerate_scale",
        "near_singular",
        "world_stack",
        "world_billiards",
        "world_card_pile",
        "batch_scale",
        "solver_parameter_sweeps",
    }
    assert all(row["benchmark_filter"] for row in benchmark_by_packet.values())
    requirement_by_name = {
        row["requirement"]: row for row in info["representative_requirement_rows"]
    }
    assert set(requirement_by_name) == {
        "Billiard symmetry, energy, momentum",
        "High mass-ratio stack",
        "Thin card pile",
        "Scalability smoke",
        "Friction coupling and active tangential bounds",
    }
    assert requirement_by_name["Billiard symmetry, energy, momentum"][
        "benchmark_packet"
    ] == "world_billiards"
    assert (
        "world_stack"
        in requirement_by_name["High mass-ratio stack"]["benchmark_packet"]
    )
    assert requirement_by_name["Thin card pile"]["live_packet"] == (
        "Thin card pile"
    )
    assert "lcp_dimension" in requirement_by_name["Scalability smoke"][
        "metrics"
    ]
    assert "contact count" in requirement_by_name[
        "Friction coupling and active tangential bounds"
    ]["metrics"]
    representative_tokens: list[str] = []
    for row in info["benchmark_packet_rows"]:
        for token in row["benchmark_filter"].split("|"):
            if token not in representative_tokens:
                representative_tokens.append(token)
    assert info["representative_benchmark_filter"].split("|") == (
        representative_tokens
    )
    assert info["representative_benchmark_command"] == (
        "pixi run bm lcp_compare -- --benchmark_filter="
        f"'{info['representative_benchmark_filter']}'"
    )
    assert info["performance_profile_refresh_command"] == (
        "pixi run python scripts/lcp_performance_profile.py --run "
        "--cache build/lcp_profile_full.json "
        "--output docs/background/lcp/figures "
        "--benchmark-timeout 900"
    )
    profile_by_surface = {
        row["surface"]: row for row in info["performance_profile_rows"]
    }
    assert set(profile_by_surface) == {"Standard", "Boxed", "FrictionIndex"}
    evidence_schema_by_fields = {
        row["fields"]: row
        for row in info["performance_profile_evidence_schema_rows"]
    }
    assert (
        "solver_identity_schema_version, solver_manifest_index"
        in evidence_schema_by_fields
    )
    assert (
        "solver_family_pivoting, solver_family_projection, "
        "solver_family_newton, solver_family_other"
        in evidence_schema_by_fields
    )
    assert (
        "solver_supports_standard, solver_supports_boxed, "
        "solver_supports_friction_index, solver_supports_problem"
        in evidence_schema_by_fields
    )
    assert "lcp_dimension, contact_count" in evidence_schema_by_fields
    assert (
        "residual, complementarity, bound_violation"
        in evidence_schema_by_fields
    )
    evidence_summary_by_surface = {
        row["surface"]: row
        for row in info["performance_profile_evidence_summary_rows"]
    }
    assert set(evidence_summary_by_surface) == {
        "Standard",
        "Boxed",
        "FrictionIndex",
    }
    assert evidence_summary_by_surface["Standard"]["rows"] == "90"
    assert evidence_summary_by_surface["Standard"]["solvers"] == "23"
    assert evidence_summary_by_surface["Standard"]["dimensions"] == (
        "2, 3, 12, 24, 48, 96"
    )
    assert evidence_summary_by_surface["Standard"]["contract_ok"] == "90/90"
    assert evidence_summary_by_surface["Boxed"]["rows"] == "45"
    assert evidence_summary_by_surface["Boxed"]["solvers"] == "15"
    assert evidence_summary_by_surface["Boxed"]["dimensions"] == "12, 24, 48"
    assert evidence_summary_by_surface["Boxed"]["contract_ok"] == "45/45"
    assert evidence_summary_by_surface["FrictionIndex"]["rows"] == "48"
    assert evidence_summary_by_surface["FrictionIndex"]["solvers"] == "16"
    assert evidence_summary_by_surface["FrictionIndex"]["dimensions"] == (
        "12, 48, 192"
    )
    assert evidence_summary_by_surface["FrictionIndex"]["contacts"] == "4, 16, 64"
    assert evidence_summary_by_surface["FrictionIndex"]["contract_ok"] == "48/48"
    solver_guidance_by_family = {
        row["family"]: row for row in info["solver_guidance_rows"]
    }
    assert set(solver_guidance_by_family) == {
        "Pivoting and direct",
        "Projection iterations",
        "Block/contact structure",
        "Newton, interior, and QP",
        "Accelerated and splitting",
    }
    assert "Dantzig" in solver_guidance_by_family["Pivoting and direct"]["solvers"]
    assert (
        "real-time approximate solves"
        in solver_guidance_by_family["Projection iterations"]["best_fit"]
    )
    assert (
        "contact piles"
        in solver_guidance_by_family["Block/contact structure"]["best_fit"]
    )
    assert (
        "high-accuracy standard rows"
        in solver_guidance_by_family["Newton, interior, and QP"]["best_fit"]
    )
    assert "rho" in solver_guidance_by_family["Accelerated and splitting"][
        "tradeoff"
    ]
    assert profile_by_surface["Standard"]["artifact"].endswith(
        "performance_profile_standard.csv"
    )
    assert profile_by_surface["Standard"]["evidence_artifact"].endswith(
        "performance_profile_evidence.csv"
    )
    assert profile_by_surface["Boxed"]["artifact"].endswith(
        "performance_profile_boxed.csv"
    )
    assert profile_by_surface["Boxed"]["evidence_artifact"].endswith(
        "performance_profile_evidence.csv"
    )
    assert profile_by_surface["FrictionIndex"]["artifact"].endswith(
        "performance_profile_frictionindex.csv"
    )
    assert profile_by_surface["FrictionIndex"]["evidence_artifact"].endswith(
        "performance_profile_evidence.csv"
    )
    assert profile_by_surface["Standard"]["problem_sizes"] == (
        "2, 3, 12, 24, 48, 96"
    )
    assert "Dantzig" in profile_by_surface["Standard"]["current_leaders"]
    assert "BGS" in profile_by_surface["Standard"]["current_leaders"]
    assert "NNCG" not in profile_by_surface["Standard"]["current_leaders"]
    assert "SubspaceMinimization" in profile_by_surface["Standard"][
        "current_leaders"
    ]
    assert "strict-interior" in profile_by_surface["Standard"][
        "current_leaders"
    ]
    assert "Newton" in profile_by_surface["Standard"]["current_leaders"]
    assert "InteriorPoint" in profile_by_surface["Standard"][
        "current_leaders"
    ]
    assert "Admm" in profile_by_surface["Standard"]["current_leaders"]
    assert "Admm" not in profile_by_surface["Standard"]["current_laggards"]
    assert "No Standard solver average is above 1.6x" in profile_by_surface[
        "Standard"
    ]["current_laggards"]
    assert "SubspaceMinimization" not in profile_by_surface["Standard"][
        "current_laggards"
    ]
    assert "InteriorPoint" not in profile_by_surface["Standard"][
        "current_laggards"
    ]
    assert "Dantzig" not in profile_by_surface["Standard"]["current_laggards"]
    assert "RedBlackGaussSeidel" in profile_by_surface["Standard"][
        "current_laggards"
    ]
    assert "Tgs" not in profile_by_surface["Standard"]["current_laggards"]
    assert "BGS" not in profile_by_surface["Standard"]["current_laggards"]
    assert "MinimumMapNewton" in profile_by_surface["Standard"][
        "current_leaders"
    ]
    assert "MPRGP" in profile_by_surface["Standard"]["current_laggards"]
    assert "Apgd" not in profile_by_surface["Standard"]["current_laggards"]
    assert "Lemke" not in profile_by_surface["Standard"]["current_laggards"]
    assert "Baraff" in profile_by_surface["Standard"]["current_laggards"]
    assert "SymmetricPsor" not in profile_by_surface["Standard"][
        "current_laggards"
    ]
    assert "NNCG" in profile_by_surface["Standard"]["current_laggards"]
    assert "Jacobi" not in profile_by_surface["Standard"]["current_laggards"]
    assert "ShockPropagation" not in profile_by_surface["Standard"][
        "current_leaders"
    ]
    assert "ShockPropagation" in profile_by_surface["Standard"][
        "current_laggards"
    ]
    assert "FischerBurmeisterNewton" in profile_by_surface["Standard"][
        "current_leaders"
    ]
    assert "Pgs/Tgs/Jacobi" in profile_by_surface["Boxed"][
        "current_leaders"
    ]
    assert "Admm" not in profile_by_surface["Boxed"]["current_leaders"]
    assert "BlockedJacobi" not in profile_by_surface["Boxed"][
        "current_leaders"
    ]
    assert "BGS" not in profile_by_surface["Boxed"]["current_leaders"]
    assert "BoxedSemiSmoothNewton" not in profile_by_surface["Boxed"][
        "current_leaders"
    ]
    assert "Dantzig" in profile_by_surface["Boxed"]["current_leaders"]
    assert "NNCG" not in profile_by_surface["Boxed"]["current_leaders"]
    assert "Sap" not in profile_by_surface["Boxed"]["current_leaders"]
    assert "ShockPropagation" not in profile_by_surface["Boxed"][
        "current_leaders"
    ]
    assert "Jacobi" in profile_by_surface["Boxed"]["current_leaders"]
    assert "SymmetricPsor" in profile_by_surface["Boxed"][
        "current_leaders"
    ]
    assert "Sap is the only Boxed solver average above 1.6x" in (
        profile_by_surface["Boxed"]["current_laggards"]
    )
    assert "Admm" in profile_by_surface["Boxed"]["current_laggards"]
    assert "BlockedJacobi" in profile_by_surface["Boxed"][
        "current_laggards"
    ]
    assert "BGS" in profile_by_surface["Boxed"]["current_laggards"]
    assert "Apgd" not in profile_by_surface["Boxed"]["current_laggards"]
    assert "RedBlackGaussSeidel" not in profile_by_surface["Boxed"][
        "current_laggards"
    ]
    assert "BoxedSemiSmoothNewton" in profile_by_surface["Boxed"][
        "current_laggards"
    ]
    assert "Dantzig" not in profile_by_surface["Boxed"]["current_laggards"]
    assert "NNCG" in profile_by_surface["Boxed"]["current_laggards"]
    assert "Sap" in profile_by_surface["Boxed"]["current_laggards"]
    assert "ShockPropagation" in profile_by_surface["Boxed"]["current_laggards"]
    assert "SymmetricPsor" not in profile_by_surface["Boxed"][
        "current_laggards"
    ]
    assert (
        "SubspaceMinimization"
        not in profile_by_surface["Boxed"]["current_leaders"]
    )
    assert "Tgs/Pgs" in profile_by_surface["FrictionIndex"][
        "current_leaders"
    ]
    assert "Admm" not in profile_by_surface["FrictionIndex"]["current_leaders"]
    assert "Apgd" in profile_by_surface["FrictionIndex"]["current_leaders"]
    assert "SymmetricPsor" not in profile_by_surface["FrictionIndex"][
        "current_leaders"
    ]
    assert "SymmetricPsor" in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "RedBlackGaussSeidel" in profile_by_surface["FrictionIndex"][
        "current_leaders"
    ]
    assert "Jacobi" in profile_by_surface["FrictionIndex"]["current_leaders"]
    assert "BGS" not in profile_by_surface["FrictionIndex"]["current_leaders"]
    assert "Dantzig" in profile_by_surface["FrictionIndex"][
        "current_leaders"
    ]
    assert "NNCG" in profile_by_surface["FrictionIndex"]["current_leaders"]
    assert "Staggering" in profile_by_surface["FrictionIndex"][
        "current_leaders"
    ]
    assert "BoxedSemiSmoothNewton" not in profile_by_surface["FrictionIndex"][
        "current_leaders"
    ]
    assert "Sap and ShockPropagation" in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "Apgd" not in profile_by_surface["FrictionIndex"]["current_laggards"]
    assert "Admm" in profile_by_surface["FrictionIndex"]["current_laggards"]
    assert "BlockedJacobi" in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "NNCG" not in profile_by_surface["FrictionIndex"]["current_laggards"]
    assert "BGS" in profile_by_surface["FrictionIndex"]["current_laggards"]
    assert "Dantzig" not in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "ShockPropagation" in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "BoxedSemiSmoothNewton" in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "SubspaceMinimization" in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "Staggering" not in profile_by_surface["FrictionIndex"][
        "current_laggards"
    ]
    assert "BGS" in profile_by_surface["FrictionIndex"]["current_laggards"]
    assert "Sap" in profile_by_surface["FrictionIndex"]["current_laggards"]
    assert benchmark_by_packet["world_billiards"]["benchmark_filter"] == (
        "BM_LcpWorldBilliardsStep_BoxedLcp"
    )
    assert benchmark_by_packet["world_stack"]["benchmark_filter"] == (
        "BM_LcpWorldStackContact/|BM_LcpWorldStackStep_BoxedLcp"
    )
    assert benchmark_by_packet["world_card_pile"]["surface"] == "boxed contact"
    assert benchmark_by_packet["world_card_pile"]["benchmark_filter"] == (
        "BM_LcpWorldCardPileStep_BoxedLcp"
    )
    assert benchmark_by_packet["active_set_scale"]["benchmark_filter"] == (
        "BM_LcpLargerActiveSetTransition|"
        "BM_LcpStressActiveSetTransition|"
        "BM_LcpExtremeActiveSetTransition|"
        "BM_LcpProductionActiveSetTransition|"
        "BM_LcpProductionActiveSetTransitionBatchSerial|"
        "BM_LcpProductionActiveSetTransitionBatchParallel"
    )
    assert benchmark_by_packet["active_friction_index_contact"]["surface"] == (
        "findex contact"
    )
    assert benchmark_by_packet["active_friction_index_contact"][
        "benchmark_filter"
    ] == "BM_LcpActiveFrictionIndexContact"
    assert benchmark_by_packet["contact_solver_comparison_sweep"][
        "benchmark_filter"
    ] == (
        "BM_LcpContactSolverComparisonSweep|"
        "BM_LcpStaggeringContactPipelineSweep"
    )
    assert benchmark_by_packet["contact_normal_standard_sweep"][
        "benchmark_filter"
    ] == "BM_LcpContactNormalStandardSweep"
    assert benchmark_by_packet["singular_degenerate_scale"][
        "benchmark_filter"
    ] == (
        "BM_LcpLargerSingularDegenerate|"
        "BM_LcpStressSingularDegenerate|"
        "BM_LcpExtremeSingularDegenerate|"
        "BM_LcpSingularDegenerateFrictionIndexBatchSerial|"
        "BM_LcpSingularDegenerateFrictionIndexBatchParallel|"
        "BM_LcpSingularDegenerateStandardBoxedBatchSerial|"
        "BM_LcpSingularDegenerateStandardBoxedBatchParallel"
    )
    assert benchmark_by_packet["solver_parameter_sweeps"][
        "benchmark_filter"
    ] == (
        "BM_LcpPgsRelaxationSweep|"
        "BM_LcpSymmetricPsorRelaxationSweep|"
        "BM_LcpRedBlackGaussSeidelRelaxationSweep|"
        "BM_LcpBoxedSemiSmoothNewtonLineSearchSweep|"
        "BM_LcpPivotingScaleSweep|"
        "BM_LcpBlockPartitionSweep|"
        "BM_LcpApgdRestartSweep|"
        "BM_LcpTgsIterationBudgetSweep|"
        "BM_LcpNncgPgsIterationsSweep|"
        "BM_LcpSubspaceMinimizationPgsIterationsSweep|"
        "BM_LcpShockPropagationLayerSweep|"
        "BM_LcpMprgpSpdCheckSweep|"
        "BM_LcpInteriorPointPathSweep|"
        "BM_LcpAdmmRhoSweep|"
        "BM_LcpSapRegularizationSweep"
    )
    builder = _FakePanelBuilder()
    setup.panels[0].build(builder, object())
    assert (
        "table:lcp_benchmark_packets:Packet,Surface,Benchmark filter,Coverage"
        in builder.events
    )
    assert (
        "table:lcp_representative_requirement_coverage:Requirement,Live packet,"
        "Benchmark packet,Metrics,Evidence"
        in builder.events
    )
    for requirement in (
        "Billiard symmetry, energy, momentum",
        "High mass-ratio stack",
        "Thin card pile",
        "Scalability smoke",
        "Friction coupling and active tangential bounds",
    ):
        assert any(requirement in event for event in builder.events)
    assert (
        "table:lcp_performance_profiles:Surface,Profile CSV,Evidence CSV,Problem sizes,"
        "Current leaders,Current laggards,Takeaway"
        in builder.events
    )
    assert (
        "table:lcp_performance_profile_evidence_schema:Field(s),Meaning"
        in builder.events
    )
    for evidence_field in (
        "solver_identity_schema_version, solver_manifest_index",
        "solver_family_pivoting, solver_family_projection, "
        "solver_family_newton, solver_family_other",
        "solver_supports_standard, solver_supports_boxed, "
        "solver_supports_friction_index, solver_supports_problem",
        "lcp_dimension, contact_count",
        "residual, complementarity, bound_violation",
    ):
        assert any(evidence_field in event for event in builder.events)
    assert (
        "table:lcp_performance_profile_evidence_summary:Surface,Rows,Solvers,"
        "LCP dimensions,Contacts,OK,Max it,Max residual,Max comp,Max bound"
        in builder.events
    )
    assert any("text:90/90" == event for event in builder.events)
    assert any("text:48/48" == event for event in builder.events)
    assert (
        "table:lcp_representative_solver_suite:Problem,Type,Rows,FI contacts,"
        "Challenge,Native,Delegated,Max residual,Fastest native"
        in builder.events
    )
    for challenge in (
        "large mass-ratio conditioning with active bounds",
        "rank-deficient complementarity with opposing active bounds",
        "two-contact active tangential bounds with coupled normals",
        "scalability smoke with banded coupling",
    ):
        assert any(challenge in event for event in builder.events)
    assert (
        "table:lcp_representative_solver_details:Problem,Solver,Route,Status,"
        "Iterations,Error,Residual,Complementarity,us"
        in builder.events
    )
    assert (
        "table:lcp_solver_selection_guide:Family,Solvers,Best fit,Strength,"
        "Tradeoff,Evidence cue"
        in builder.events
    )
    for guidance_text in (
        "Pivoting and direct",
        "Projection iterations",
        "Block/contact structure",
        "Newton, interior, and QP",
        "Accelerated and splitting",
    ):
        assert any(guidance_text in event for event in builder.events)
    assert (
        "table:lcp_solver_profile:Solver,Native cases,OK,Total us,Worst error,"
        "Worst residual,Slowest case"
        in builder.events
    )
    assert (
        "table:lcp_advanced_solver_parameters:Solver,Surface,Parameters,Defaults,"
        "Benchmark"
        in builder.events
    )
    for plot_prefix in (
        "plot:Sequential billiard momentum error:",
        "plot:Boxed LCP billiard momentum error:",
        "plot:Sequential billiard energy error:",
        "plot:Boxed LCP billiard energy error:",
        "plot:Sequential billiard symmetry error:",
        "plot:Boxed LCP billiard symmetry error:",
    ):
        assert any(event.startswith(plot_prefix) for event in builder.events)


def test_lcp_physics_updates_live_metrics_headlessly() -> None:
    _require_simulation_symbols("World", "ContactSolverMethod")

    setup = lcp_physics.build()
    assert setup.pre_step is not None

    snapshot = setup.info["live_metrics_snapshot"]
    assert callable(snapshot)
    before = snapshot()
    assert set(before) == {"Sequential impulse", "Boxed LCP"}
    assert all(row["step_samples"] == 0 for row in before.values())
    assert all(row["contact_samples"] == 0 for row in before.values())

    step_count = 5
    for _ in range(step_count):
        setup.pre_step()

    nonnegative_metrics = {
        "step_ms",
        "contacts",
        "sliding_speed",
        "billiard_momentum_error",
        "billiard_energy_error",
        "billiard_symmetry_error",
        "stack_lateral_drift",
        "card_spread",
        "card_height_loss",
    }
    finite_metrics = nonnegative_metrics | {"ramp_slide"}
    after = snapshot()
    assert set(after) == {"Sequential impulse", "Boxed LCP"}
    for row in after.values():
        assert row["step_samples"] == step_count
        assert row["contact_samples"] == step_count
        for metric in finite_metrics:
            assert np.isfinite(row[metric]), metric
        for metric in nonnegative_metrics:
            assert row[metric] >= 0.0, metric
        assert row["sliding_speed"] > 0.0


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
        assert any(
            event.startswith("text:smoke: pixi run py-demos")
            for event in builder.events
        )
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
    assert any(
        event.startswith("text:swinging tip radius: ") for event in builder.events
    )


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
    assert "text:benchmark: pixi run bm-plan083-cpu-lying-flat-packet" in builder.events
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
