"""PLAN-083 CPU corpus placeholders for unified Newton-barrier scenes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import dartpy as dart
import numpy as np

from .._ipc_deformable_bridge import IpcDeformableBridge, build_grid_options
from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BRIDGE_BOARD_HALF_EXTENTS = np.array([0.10, 0.16, 0.025])
_BRIDGE_POST_HALF_EXTENTS = np.array([0.05, 0.2, 0.08])
_BRIDGE_TRAVELER_HALF_EXTENTS = np.array([0.07, 0.07, 0.07])
_BRIDGE_BOARD_X = np.linspace(-0.45, 0.45, 4)
_PULLEY_SUPPORT_HALF_EXTENTS = np.array([0.08, 0.04, 0.08])
_PULLEY_WHEEL_RADIUS = 0.10
_PULLEY_LOAD_HALF_EXTENTS = np.array([0.055, 0.055, 0.055])
_UMBRELLA_MAST_HALF_EXTENTS = np.array([0.04, 0.04, 0.28])
_UMBRELLA_HUB_HALF_EXTENTS = np.array([0.05, 0.05, 0.04])
_UMBRELLA_RIB_HALF_EXTENTS = np.array([0.18, 0.025, 0.025])
_LYING_FLAT_GROUND_HALF_EXTENTS = np.array([0.28, 0.18, 0.012])
_LYING_FLAT_ROD_HALF_EXTENTS = np.array([0.14, 0.015, 0.015])
_LYING_FLAT_RING_RADIUS = 0.028
_LYING_FLAT_GRID_COLUMNS = 6
_LYING_FLAT_GRID_ROWS = 4
_LYING_FLAT_GRID_SPACING = 0.055
_CANDY_SHELL_HALF_EXTENTS = np.array([0.24, 0.16, 0.025])
_CANDY_GRID_COLUMNS = 5
_CANDY_GRID_ROWS = 5
_CANDY_GRID_SPACING = 0.055
_NUNCHAKU_HANDLE_HALF_EXTENTS = np.array([0.18, 0.035, 0.035])
_TERRAIN_HALF_EXTENTS = np.array([0.70, 0.45, 0.025])
_TERRAIN_CHASSIS_HALF_EXTENTS = np.array([0.22, 0.12, 0.04])
_TERRAIN_WHEEL_RADIUS = 0.06
_PRECESSION_GROUND_HALF_EXTENTS = np.array([0.55, 0.45, 0.025])
_PRECESSION_WHEEL_RADIUS = 0.16
_PRECESSION_WHEEL_HALF_HEIGHT = 0.035
_RAGDOLL_GROUND_HALF_EXTENTS = np.array([0.42, 0.36, 0.025])
_RAGDOLL_TORSO_HALF_EXTENTS = np.array([0.10, 0.07, 0.14])
_RAGDOLL_ARM_HALF_EXTENTS = np.array([0.12, 0.025, 0.035])
_RAGDOLL_LEG_HALF_EXTENTS = np.array([0.035, 0.04, 0.16])
_RAGDOLL_HEAD_RADIUS = 0.055
_TERRAIN_WHEEL_OFFSETS = (
    np.array([-0.16, -0.14, -0.10]),
    np.array([-0.16, 0.14, -0.10]),
    np.array([0.16, -0.14, -0.10]),
    np.array([0.16, 0.14, -0.10]),
)
_RAGDOLL_PARTS = (
    ("head", np.array([0.00, 0.0, 0.62]), "sphere"),
    ("left_arm", np.array([-0.19, 0.0, 0.45]), "arm"),
    ("right_arm", np.array([0.19, 0.0, 0.45]), "arm"),
    ("left_leg", np.array([-0.07, 0.0, 0.16]), "leg"),
    ("right_leg", np.array([0.07, 0.0, 0.16]), "leg"),
)
_WINDMILL_HUB_HALF_EXTENTS = np.array([0.05, 0.05, 0.05])
_WINDMILL_BLADE_HALF_EXTENTS = np.array([0.045, 0.22, 0.025])
_WINDMILL_STRIKER_HALF_EXTENTS = np.array([0.09, 0.09, 0.09])


@dataclass(frozen=True)
class Plan083SceneTarget:
    scene_id: str
    title: str
    row_ids: tuple[str, ...]
    category: str
    summary: str
    target: str
    smoke_command: str
    visual_command: str
    benchmark_command: str
    limitation: str


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _quat_x(angle: float) -> tuple[float, float, float, float]:
    half = 0.5 * angle
    return (float(np.cos(half)), float(np.sin(half)), 0.0, 0.0)


def _target_info(target: Plan083SceneTarget) -> dict[str, object]:
    return {
        "plan083_cpu_corpus_scene": target.scene_id,
        "plan083_row_ids": target.row_ids,
        "plan083_smoke_command": target.smoke_command,
        "plan083_visual_command": target.visual_command,
        "plan083_benchmark_command": target.benchmark_command,
        "plan083_limitation": target.limitation,
    }


def _build_placeholder(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.gui.DescriptorRenderScene(dart.World(), target.scene_id)
    world.set_time_step(1.0 / 60.0)

    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        f"{target.scene_id}_marker",
        _translation(0.0, 0.0, 0.1),
    )
    frame.set_shape(dart.BoxShape(np.array([0.68, 0.44, 0.2])))
    frame.create_visual_aspect().set_color([0.22, 0.45, 0.65])
    world.add_simple_frame(frame)

    row_ids = ", ".join(target.row_ids)

    def build_panel(builder: object, context: object) -> None:
        builder.text("status: planned PLAN-083 CPU corpus scene")
        builder.text(f"rows: {row_ids}")
        builder.separator()
        builder.text(f"target: {target.target}")
        builder.separator()
        builder.text(f"smoke: {target.smoke_command}")
        builder.text(f"visual: {target.visual_command}")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.separator()
        builder.text(f"limitation: {target.limitation}")

    return SceneSetup(
        world=world,
        panels=[ScenePanel(target.title, build_panel)],
        info=_target_info(target),
    )


def _build_hanging_bridge_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, -9.81),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    left_post = world.add_rigid_body(
        "plan083_bridge_left_post", position=(-0.65, 0.0, 0.56)
    )
    left_post.is_static = True
    left_post.set_collision_shape(dart.CollisionShape.box(_BRIDGE_POST_HALF_EXTENTS))

    right_post = world.add_rigid_body(
        "plan083_bridge_right_post", position=(0.65, 0.0, 0.56)
    )
    right_post.is_static = True
    right_post.set_collision_shape(dart.CollisionShape.box(_BRIDGE_POST_HALF_EXTENTS))

    boards = []
    parent = left_post
    for index, x in enumerate(_BRIDGE_BOARD_X):
        board = world.add_rigid_body(
            f"plan083_bridge_board_{index}",
            position=(float(x), 0.0, 0.50),
        )
        board.is_static = True
        board.mass = 0.25
        board.friction = 0.7
        board.set_collision_shape(dart.CollisionShape.box(_BRIDGE_BOARD_HALF_EXTENTS))
        world.add_rigid_body_fixed_joint(
            f"plan083_bridge_point_connection_{index}",
            parent,
            board,
        )
        boards.append(board)
        parent = board

    traveler = world.add_rigid_body(
        "plan083_bridge_traveler",
        position=(-0.60, 0.0, 0.82),
        linear_velocity=(0.35, 0.0, -0.35),
    )
    traveler.is_kinematic = True
    traveler.mass = 0.12
    traveler.friction = 0.4
    traveler.set_collision_shape(dart.CollisionShape.box(_BRIDGE_TRAVELER_HALF_EXTENTS))

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_hanging_bridge_runtime")
    bridge.add_rigid_body_visual(
        left_post,
        dart.BoxShape(_full(_BRIDGE_POST_HALF_EXTENTS)),
        (0.32, 0.34, 0.38),
        name="plan083_bridge_left_post_visual",
    )
    bridge.add_rigid_body_visual(
        right_post,
        dart.BoxShape(_full(_BRIDGE_POST_HALF_EXTENTS)),
        (0.32, 0.34, 0.38),
        name="plan083_bridge_right_post_visual",
    )
    board_palette = (
        (0.72, 0.46, 0.22),
        (0.76, 0.50, 0.25),
    )
    for index, board in enumerate(boards):
        bridge.add_rigid_body_visual(
            board,
            dart.BoxShape(_full(_BRIDGE_BOARD_HALF_EXTENTS)),
            board_palette[index % len(board_palette)],
            name=f"plan083_bridge_board_{index}_visual",
        )
    bridge.add_rigid_body_visual(
        traveler,
        dart.BoxShape(_full(_BRIDGE_TRAVELER_HALF_EXTENTS)),
        (0.18, 0.50, 0.84),
        name="plan083_bridge_traveler_visual",
    )
    bridge.sync()

    traveler_height_history: deque[float] = deque(maxlen=120)
    traveler_x_history: deque[float] = deque(maxlen=120)
    board_sag_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        traveler_position = np.asarray(traveler.translation, dtype=float).reshape(3)
        board_heights = [
            float(np.asarray(board.translation, dtype=float).reshape(3)[2])
            for board in boards
        ]
        traveler_height_history.append(float(traveler_position[2]))
        traveler_x_history.append(float(traveler_position[0]))
        board_sag_history.append(0.50 - min(board_heights))

        builder.text("status: runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"point connections: {world.num_rigid_body_fixed_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"traveler x: {traveler_position[0]:.3f} m")
        builder.text(f"traveler height: {traveler_position[2]:.3f} m")
        builder.text(f"max board sag: {board_sag_history[-1]:.4f} m")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if traveler_height_history:
            builder.separator()
            builder.plot_lines("Traveler height", list(traveler_height_history))
            builder.plot_lines("Traveler x", list(traveler_x_history))
            builder.plot_lines("Board sag", list(board_sag_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "boards": tuple(boards),
            "traveler": traveler,
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_nunchaku_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, 0.0),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    anchor = world.add_rigid_body(
        "plan083_nunchaku_anchor_handle", position=(0.0, 0.0, 0.75)
    )
    anchor.is_static = True
    anchor.set_collision_shape(
        dart.CollisionShape.box(
            _NUNCHAKU_HANDLE_HALF_EXTENTS,
            _translation(-float(_NUNCHAKU_HANDLE_HALF_EXTENTS[0]), 0.0, 0.0),
        )
    )

    swinging = world.add_rigid_body(
        "plan083_nunchaku_swing_handle",
        position=(0.0, 0.0, 0.75),
        angular_velocity=(0.0, 0.0, 1.5),
    )
    swinging.mass = 0.2
    swinging.is_kinematic = True
    swinging.set_collision_shape(
        dart.CollisionShape.box(
            _NUNCHAKU_HANDLE_HALF_EXTENTS,
            _translation(float(_NUNCHAKU_HANDLE_HALF_EXTENTS[0]), 0.0, 0.0),
        )
    )
    world.add_rigid_body_revolute_joint(
        "plan083_nunchaku_hinge",
        anchor,
        swinging,
        axis=(0.0, 0.0, 1.0),
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_nunchaku_runtime")
    bridge.add_rigid_body_visual(
        anchor,
        dart.BoxShape(_full(_NUNCHAKU_HANDLE_HALF_EXTENTS)),
        (0.38, 0.33, 0.29),
        name="plan083_nunchaku_anchor_visual",
        local_transform=_translation(-float(_NUNCHAKU_HANDLE_HALF_EXTENTS[0]), 0.0, 0.0),
    )
    bridge.add_rigid_body_visual(
        swinging,
        dart.BoxShape(_full(_NUNCHAKU_HANDLE_HALF_EXTENTS)),
        (0.88, 0.56, 0.20),
        name="plan083_nunchaku_swing_visual",
        local_transform=_translation(float(_NUNCHAKU_HANDLE_HALF_EXTENTS[0]), 0.0, 0.0),
    )
    bridge.sync()

    tip_radius_history: deque[float] = deque(maxlen=120)
    angular_velocity_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        transform = np.asarray(swinging.transform, dtype=float).reshape(4, 4)
        local_tip = np.array([2.0 * _NUNCHAKU_HANDLE_HALF_EXTENTS[0], 0.0, 0.0, 1.0])
        tip = transform @ local_tip
        anchor_position = np.asarray(anchor.translation, dtype=float).reshape(3)
        tip_radius = float(np.linalg.norm(tip[:3] - anchor_position))
        angular_velocity = float(np.asarray(swinging.angular_velocity, dtype=float)[2])
        tip_radius_history.append(tip_radius)
        angular_velocity_history.append(angular_velocity)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"revolute joints: {world.num_rigid_body_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"swinging tip radius: {tip_radius:.3f} m")
        builder.text(f"free-axis angular velocity: {angular_velocity:.3f} rad/s")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if tip_radius_history:
            builder.separator()
            builder.plot_lines("Swinging tip radius", list(tip_radius_history))
            builder.plot_lines(
                "Free-axis angular velocity", list(angular_velocity_history)
            )

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "anchor": anchor,
            "swinging": swinging,
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_pulley_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, -9.81),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    support = world.add_rigid_body(
        "plan083_pulley_support", position=(0.0, 0.0, 0.76)
    )
    support.is_static = True
    support.set_collision_shape(dart.CollisionShape.box(_PULLEY_SUPPORT_HALF_EXTENTS))

    wheel = world.add_rigid_body(
        "plan083_pulley_wheel",
        position=(0.0, 0.0, 0.76),
    )
    wheel.mass = 0.16
    wheel.set_collision_shape(dart.CollisionShape.sphere(_PULLEY_WHEEL_RADIUS))

    left_load = world.add_rigid_body(
        "plan083_pulley_left_load",
        position=(-0.20, 0.0, 0.48),
    )
    left_load.mass = 0.12
    left_load.set_collision_shape(dart.CollisionShape.box(_PULLEY_LOAD_HALF_EXTENTS))

    right_load = world.add_rigid_body(
        "plan083_pulley_right_load",
        position=(0.20, 0.0, 0.42),
    )
    right_load.mass = 0.18
    right_load.set_collision_shape(dart.CollisionShape.box(_PULLEY_LOAD_HALF_EXTENTS))

    world.add_rigid_body_revolute_joint(
        "plan083_pulley_hinge",
        support,
        wheel,
        axis=(0.0, 1.0, 0.0),
    )
    world.add_rigid_body_fixed_joint(
        "plan083_pulley_left_point_connection",
        wheel,
        left_load,
    )
    world.add_rigid_body_fixed_joint(
        "plan083_pulley_right_point_connection",
        wheel,
        right_load,
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_pulley_runtime")
    bridge.add_rigid_body_visual(
        support,
        dart.BoxShape(_full(_PULLEY_SUPPORT_HALF_EXTENTS)),
        (0.36, 0.37, 0.40),
        name="plan083_pulley_support_visual",
    )
    bridge.add_rigid_body_visual(
        wheel,
        dart.SphereShape(_PULLEY_WHEEL_RADIUS),
        (0.22, 0.52, 0.76),
        name="plan083_pulley_wheel_visual",
    )
    bridge.add_rigid_body_visual(
        left_load,
        dart.BoxShape(_full(_PULLEY_LOAD_HALF_EXTENTS)),
        (0.74, 0.45, 0.24),
        name="plan083_pulley_left_load_visual",
    )
    bridge.add_rigid_body_visual(
        right_load,
        dart.BoxShape(_full(_PULLEY_LOAD_HALF_EXTENTS)),
        (0.86, 0.32, 0.26),
        name="plan083_pulley_right_load_visual",
    )
    bridge.sync()

    left_height_history: deque[float] = deque(maxlen=120)
    right_height_history: deque[float] = deque(maxlen=120)
    wheel_spin_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        left_height = float(np.asarray(left_load.translation, dtype=float)[2])
        right_height = float(np.asarray(right_load.translation, dtype=float)[2])
        wheel_spin = float(np.asarray(wheel.angular_velocity, dtype=float)[1])
        left_height_history.append(left_height)
        right_height_history.append(right_height)
        wheel_spin_history.append(wheel_spin)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"joints: {world.num_rigid_body_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"left load height: {left_height:.3f} m")
        builder.text(f"right load height: {right_height:.3f} m")
        builder.text(f"wheel spin: {wheel_spin:.3f} rad/s")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if left_height_history:
            builder.separator()
            builder.plot_lines("Left load height", list(left_height_history))
            builder.plot_lines("Right load height", list(right_height_history))
            builder.plot_lines("Wheel spin", list(wheel_spin_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "support": support,
            "wheel": wheel,
            "loads": (left_load, right_load),
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_umbrella_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, 0.0),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    mast = world.add_rigid_body("plan083_umbrella_mast", position=(0.0, 0.0, 0.48))
    mast.is_static = True
    mast.set_collision_shape(dart.CollisionShape.box(_UMBRELLA_MAST_HALF_EXTENTS))

    hub = world.add_rigid_body(
        "plan083_umbrella_hinged_hub",
        position=(0.0, 0.0, 0.78),
        angular_velocity=(0.0, 0.9, 0.0),
    )
    hub.mass = 0.12
    hub.set_collision_shape(dart.CollisionShape.box(_UMBRELLA_HUB_HALF_EXTENTS))

    left_rib = world.add_rigid_body(
        "plan083_umbrella_left_rib",
        position=(-0.18, 0.0, 0.74),
        angular_velocity=(0.0, 0.9, 0.0),
    )
    left_rib.mass = 0.08
    left_rib.set_collision_shape(dart.CollisionShape.box(_UMBRELLA_RIB_HALF_EXTENTS))

    right_rib = world.add_rigid_body(
        "plan083_umbrella_right_rib",
        position=(0.18, 0.0, 0.74),
        angular_velocity=(0.0, 0.9, 0.0),
    )
    right_rib.mass = 0.08
    right_rib.set_collision_shape(dart.CollisionShape.box(_UMBRELLA_RIB_HALF_EXTENTS))

    world.add_rigid_body_revolute_joint(
        "plan083_umbrella_canopy_hinge",
        mast,
        hub,
        axis=(0.0, 1.0, 0.0),
    )
    world.add_rigid_body_fixed_joint(
        "plan083_umbrella_left_rib_point_connection",
        hub,
        left_rib,
    )
    world.add_rigid_body_fixed_joint(
        "plan083_umbrella_right_rib_point_connection",
        hub,
        right_rib,
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_umbrella_runtime")
    bridge.add_rigid_body_visual(
        mast,
        dart.BoxShape(_full(_UMBRELLA_MAST_HALF_EXTENTS)),
        (0.34, 0.35, 0.38),
        name="plan083_umbrella_mast_visual",
    )
    bridge.add_rigid_body_visual(
        hub,
        dart.BoxShape(_full(_UMBRELLA_HUB_HALF_EXTENTS)),
        (0.18, 0.48, 0.72),
        name="plan083_umbrella_hub_visual",
    )
    bridge.add_rigid_body_visual(
        left_rib,
        dart.BoxShape(_full(_UMBRELLA_RIB_HALF_EXTENTS)),
        (0.76, 0.42, 0.22),
        name="plan083_umbrella_left_rib_visual",
    )
    bridge.add_rigid_body_visual(
        right_rib,
        dart.BoxShape(_full(_UMBRELLA_RIB_HALF_EXTENTS)),
        (0.76, 0.42, 0.22),
        name="plan083_umbrella_right_rib_visual",
    )
    bridge.sync()

    span_history: deque[float] = deque(maxlen=120)
    hinge_velocity_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        left_position = np.asarray(left_rib.translation, dtype=float).reshape(3)
        right_position = np.asarray(right_rib.translation, dtype=float).reshape(3)
        span = float(np.linalg.norm(right_position - left_position))
        hinge_velocity = float(np.asarray(hub.angular_velocity, dtype=float)[1])
        span_history.append(span)
        hinge_velocity_history.append(hinge_velocity)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text("revolute joints: 1")
        builder.text(f"point connections: {world.num_rigid_body_fixed_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"canopy span: {span:.3f} m")
        builder.text(f"hinge velocity: {hinge_velocity:.3f} rad/s")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if span_history:
            builder.separator()
            builder.plot_lines("Canopy span", list(span_history))
            builder.plot_lines("Hinge velocity", list(hinge_velocity_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "mast": mast,
            "hub": hub,
            "ribs": (left_rib, right_rib),
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_lying_flat_runtime(target: Plan083SceneTarget) -> SceneSetup:
    columns = _LYING_FLAT_GRID_COLUMNS
    rows = _LYING_FLAT_GRID_ROWS
    spacing = _LYING_FLAT_GRID_SPACING
    half_width = 0.5 * spacing * (columns - 1)
    half_depth = 0.5 * spacing * (rows - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        stagger = 0.003 if (col + row) % 2 else 0.0
        return (spacing * col - half_width, spacing * row - half_depth, 0.078 + stagger)

    def velocity(col: int, row: int) -> tuple[float, float, float]:
        lateral = 0.025 if col < columns // 2 else -0.025
        return (lateral, 0.0, -0.02)

    options = build_grid_options(
        columns,
        rows,
        position_fn=position,
        velocity_fn=velocity,
        mass=0.02,
        edge_stiffness=80.0,
        damping=1.4,
    )
    material = options.material
    material.friction_coefficient = 0.35
    options.material = material

    world = dart.World(time_step=0.004, gravity=(0.0, 0.0, -1.0))

    def add_obstacle(
        name: str,
        center: tuple[float, float, float],
        shape: object,
    ) -> object:
        body = world.add_rigid_body(name, position=center)
        body.is_static = True
        body.set_collision_shape(shape)
        body.is_deformable_surface_ccd_obstacle = True
        body.is_deformable_obstacle_barrier_only = True
        return body

    ground = add_obstacle(
        "plan083_lying_flat_ground",
        (0.0, 0.0, _LYING_FLAT_GROUND_HALF_EXTENTS[2]),
        dart.CollisionShape.box(_LYING_FLAT_GROUND_HALF_EXTENTS),
    )
    rod = add_obstacle(
        "plan083_lying_flat_reduced_rod",
        (0.0, 0.055, 0.044),
        dart.CollisionShape.box(_LYING_FLAT_ROD_HALF_EXTENTS),
    )
    left_ring = add_obstacle(
        "plan083_lying_flat_left_ring",
        (-0.09, -0.045, 0.042),
        dart.CollisionShape.sphere(_LYING_FLAT_RING_RADIUS),
    )
    right_ring = add_obstacle(
        "plan083_lying_flat_right_ring",
        (0.09, -0.045, 0.042),
        dart.CollisionShape.sphere(_LYING_FLAT_RING_RADIUS),
    )
    cloth = world.add_deformable_body("plan083_lying_flat_deformable_cloth", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="plan083_lying_flat_runtime")
    bridge.add_rigid_box_visual(
        (0.0, 0.0, _LYING_FLAT_GROUND_HALF_EXTENTS[2]),
        tuple(_full(_LYING_FLAT_GROUND_HALF_EXTENTS)),
        (0.36, 0.42, 0.44),
        name="plan083_lying_flat_ground_visual",
    )
    bridge.add_rigid_box_visual(
        (0.0, 0.055, 0.044),
        tuple(_full(_LYING_FLAT_ROD_HALF_EXTENTS)),
        (0.78, 0.58, 0.32),
        name="plan083_lying_flat_rod_visual",
    )
    bridge.add_rigid_sphere_visual(
        (-0.09, -0.045, 0.042),
        _LYING_FLAT_RING_RADIUS,
        (0.62, 0.44, 0.76),
        name="plan083_lying_flat_left_ring_visual",
    )
    bridge.add_rigid_sphere_visual(
        (0.09, -0.045, 0.042),
        _LYING_FLAT_RING_RADIUS,
        (0.62, 0.44, 0.76),
        name="plan083_lying_flat_right_ring_visual",
    )
    bridge.add_deformable_visual(cloth, name="plan083_lying_flat_cloth")

    height_history: deque[float] = deque(maxlen=120)
    contact_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = [
            np.asarray(cloth.node_position(index), dtype=float).reshape(3)
            for index in range(cloth.node_count)
        ]
        mean_height = float(np.mean([point[2] for point in positions]))
        diagnostics = world.last_deformable_solver_diagnostics
        active_contacts = float(diagnostics.converged_active_contact_count)
        height_history.append(mean_height)
        contact_history.append(active_contacts)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: deformable IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid obstacles: {world.num_rigid_bodies}")
        builder.text(f"deformable bodies: {world.num_deformable_bodies}")
        builder.text(f"nodes: {cloth.node_count}")
        builder.text(f"surface triangles: {cloth.surface_triangle_count}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"mean cloth height: {mean_height:.3f} m")
        builder.text(f"active contacts: {active_contacts:.0f}")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if height_history:
            builder.separator()
            builder.plot_lines("Mean cloth height", list(height_history))
            builder.plot_lines("Active contacts", list(contact_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "deformable_solver": "ipc",
            "cloth": cloth,
            "obstacles": (ground, rod, left_ring, right_ring),
        }
    )

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_candy_runtime(target: Plan083SceneTarget) -> SceneSetup:
    columns = _CANDY_GRID_COLUMNS
    rows = _CANDY_GRID_ROWS
    spacing = _CANDY_GRID_SPACING
    half_width = 0.5 * spacing * (columns - 1)
    half_depth = 0.5 * spacing * (rows - 1)

    def position(col: int, row: int) -> tuple[float, float, float]:
        return (spacing * col - half_width, spacing * row - half_depth, 0.075)

    def velocity(col: int, row: int) -> tuple[float, float, float]:
        lateral = 0.08 if row % 2 else -0.08
        return (lateral, 0.0, -0.05)

    options = build_grid_options(
        columns,
        rows,
        position_fn=position,
        velocity_fn=velocity,
        mass=0.025,
        edge_stiffness=90.0,
        damping=1.2,
    )
    material = options.material
    material.friction_coefficient = 0.45
    options.material = material

    world = dart.World(time_step=0.004, gravity=(0.0, 0.0, -1.5))
    shell = world.add_rigid_body("plan083_candy_reduced_shell", position=(0.0, 0.0, 0.025))
    shell.is_static = True
    shell.set_collision_shape(dart.CollisionShape.box(_CANDY_SHELL_HALF_EXTENTS))
    shell.is_deformable_surface_ccd_obstacle = True
    shell.is_deformable_obstacle_barrier_only = True
    cloth = world.add_deformable_body("plan083_candy_deformable_cloth", options)
    world.enter_simulation_mode()

    bridge = IpcDeformableBridge(world, name="plan083_candy_runtime")
    bridge.add_rigid_box_visual(
        (0.0, 0.0, 0.025),
        tuple(_full(_CANDY_SHELL_HALF_EXTENTS)),
        (0.55, 0.38, 0.68),
        name="plan083_candy_shell_visual",
    )
    bridge.add_deformable_visual(cloth, name="plan083_candy_cloth")

    height_history: deque[float] = deque(maxlen=120)
    contact_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        positions = [
            np.asarray(cloth.node_position(index), dtype=float).reshape(3)
            for index in range(cloth.node_count)
        ]
        mean_height = float(np.mean([point[2] for point in positions]))
        diagnostics = world.last_deformable_solver_diagnostics
        active_contacts = float(diagnostics.converged_active_contact_count)
        height_history.append(mean_height)
        contact_history.append(active_contacts)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: deformable IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"deformable bodies: {world.num_deformable_bodies}")
        builder.text(f"nodes: {cloth.node_count}")
        builder.text(f"surface triangles: {cloth.surface_triangle_count}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"mean cloth height: {mean_height:.3f} m")
        builder.text(f"active contacts: {active_contacts:.0f}")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_diagnostics_panel(builder, context)
        if height_history:
            builder.separator()
            builder.plot_lines("Mean cloth height", list(height_history))
            builder.plot_lines("Active contacts", list(contact_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "deformable_solver": "ipc",
            "cloth": cloth,
            "shell": shell,
        }
    )

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_windmill_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, -9.81),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    hub = world.add_rigid_body("plan083_windmill_hub", position=(0.0, 0.0, 0.65))
    hub.is_static = True
    hub.set_collision_shape(dart.CollisionShape.box(_WINDMILL_HUB_HALF_EXTENTS))

    blade = world.add_rigid_body(
        "plan083_windmill_blade",
        position=(0.0, 0.0, 0.65),
        angular_velocity=(0.0, 1.2, 0.0),
    )
    blade.mass = 0.25
    blade.is_kinematic = True
    blade.friction = 0.6
    blade.set_collision_shape(
        dart.CollisionShape.box(
            _WINDMILL_BLADE_HALF_EXTENTS,
            _translation(0.0, 0.0, 0.18),
        )
    )

    striker = world.add_rigid_body(
        "plan083_windmill_falling_box",
        position=(0.0, 0.0, 0.955),
        linear_velocity=(0.0, 0.0, -0.25),
    )
    striker.mass = 0.18
    striker.friction = 0.6
    striker.set_collision_shape(dart.CollisionShape.box(_WINDMILL_STRIKER_HALF_EXTENTS))

    world.add_rigid_body_revolute_joint(
        "plan083_windmill_hinge",
        hub,
        blade,
        axis=(0.0, 1.0, 0.0),
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_windmill_runtime")
    bridge.add_rigid_body_visual(
        hub,
        dart.BoxShape(_full(_WINDMILL_HUB_HALF_EXTENTS)),
        (0.36, 0.38, 0.42),
        name="plan083_windmill_hub_visual",
    )
    bridge.add_rigid_body_visual(
        blade,
        dart.BoxShape(_full(_WINDMILL_BLADE_HALF_EXTENTS)),
        (0.25, 0.62, 0.72),
        name="plan083_windmill_blade_visual",
        local_transform=_translation(0.0, 0.0, 0.18),
    )
    bridge.add_rigid_body_visual(
        striker,
        dart.BoxShape(_full(_WINDMILL_STRIKER_HALF_EXTENTS)),
        (0.86, 0.42, 0.28),
        name="plan083_windmill_striker_visual",
    )
    bridge.sync()

    blade_tip_history: deque[float] = deque(maxlen=120)
    striker_height_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        blade_transform = np.asarray(blade.transform, dtype=float).reshape(4, 4)
        local_tip = np.array([0.0, 0.0, 0.36, 1.0])
        tip = blade_transform @ local_tip
        hub_position = np.asarray(hub.translation, dtype=float).reshape(3)
        striker_position = np.asarray(striker.translation, dtype=float).reshape(3)
        tip_radius = float(np.linalg.norm(tip[:3] - hub_position))
        blade_tip_history.append(tip_radius)
        striker_height_history.append(float(striker_position[2]))

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"revolute joints: {world.num_rigid_body_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"blade tip radius: {tip_radius:.3f} m")
        builder.text(f"striker height: {striker_position[2]:.3f} m")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if blade_tip_history:
            builder.separator()
            builder.plot_lines("Blade tip radius", list(blade_tip_history))
            builder.plot_lines("Striker height", list(striker_height_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "hub": hub,
            "blade": blade,
            "striker": striker,
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_terrain_vehicle_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, -9.81),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    terrain = world.add_rigid_body("plan083_vehicle_terrain", position=(0.0, 0.0, -0.025))
    terrain.is_static = True
    terrain.friction = 0.8
    terrain.set_collision_shape(dart.CollisionShape.box(_TERRAIN_HALF_EXTENTS))

    chassis = world.add_rigid_body(
        "plan083_vehicle_chassis",
        position=(0.0, 0.0, 0.17),
        linear_velocity=(0.12, 0.0, 0.0),
    )
    chassis.is_kinematic = True
    chassis.mass = 0.45
    chassis.friction = 0.7
    chassis.set_collision_shape(dart.CollisionShape.box(_TERRAIN_CHASSIS_HALF_EXTENTS))

    wheels = []
    chassis_position = np.asarray(chassis.translation, dtype=float).reshape(3)
    for index, offset in enumerate(_TERRAIN_WHEEL_OFFSETS):
        wheel = world.add_rigid_body(
            f"plan083_vehicle_passive_wheel_{index}",
            position=tuple(chassis_position + offset),
            linear_velocity=(0.12, 0.0, 0.0),
            angular_velocity=(0.0, 3.0, 0.0),
        )
        wheel.is_kinematic = True
        wheel.mass = 0.08
        wheel.friction = 0.9
        wheel.set_collision_shape(dart.CollisionShape.sphere(_TERRAIN_WHEEL_RADIUS))
        world.add_rigid_body_revolute_joint(
            f"plan083_vehicle_wheel_hinge_{index}",
            chassis,
            wheel,
            axis=(0.0, 1.0, 0.0),
        )
        wheels.append(wheel)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_terrain_vehicle_runtime")
    bridge.add_rigid_body_visual(
        terrain,
        dart.BoxShape(_full(_TERRAIN_HALF_EXTENTS)),
        (0.34, 0.43, 0.32),
        name="plan083_vehicle_terrain_visual",
    )
    bridge.add_rigid_body_visual(
        chassis,
        dart.BoxShape(_full(_TERRAIN_CHASSIS_HALF_EXTENTS)),
        (0.24, 0.42, 0.78),
        name="plan083_vehicle_chassis_visual",
    )
    for index, wheel in enumerate(wheels):
        bridge.add_rigid_body_visual(
            wheel,
            dart.SphereShape(_TERRAIN_WHEEL_RADIUS),
            (0.10, 0.12, 0.14),
            name=f"plan083_vehicle_wheel_{index}_visual",
        )
    bridge.sync()

    chassis_height_history: deque[float] = deque(maxlen=120)
    min_clearance_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        chassis_position = np.asarray(chassis.translation, dtype=float).reshape(3)
        wheel_clearances = [
            float(np.asarray(wheel.translation, dtype=float).reshape(3)[2])
            - _TERRAIN_WHEEL_RADIUS
            for wheel in wheels
        ]
        min_clearance = min(wheel_clearances)
        chassis_height_history.append(float(chassis_position[2]))
        min_clearance_history.append(min_clearance)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"passive wheels: {len(wheels)}")
        builder.text(f"revolute joints: {world.num_rigid_body_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"chassis height: {chassis_position[2]:.3f} m")
        builder.text(f"min wheel clearance: {min_clearance:.4f} m")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if chassis_height_history:
            builder.separator()
            builder.plot_lines("Chassis height", list(chassis_height_history))
            builder.plot_lines("Min wheel clearance", list(min_clearance_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "terrain": terrain,
            "chassis": chassis,
            "wheels": tuple(wheels),
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_precession_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, -9.81),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    ground = world.add_rigid_body(
        "plan083_precession_ground", position=(0.0, 0.0, -0.025)
    )
    ground.is_static = True
    ground.friction = 0.8
    ground.set_collision_shape(dart.CollisionShape.box(_PRECESSION_GROUND_HALF_EXTENTS))

    wheel = world.add_rigid_body(
        "plan083_precession_wheel",
        position=(-0.18, 0.0, _PRECESSION_WHEEL_RADIUS),
        orientation=_quat_x(-0.5 * np.pi),
        linear_velocity=(0.28, 0.0, 0.0),
        angular_velocity=(0.0, 8.0, 1.2),
    )
    wheel.mass = 0.22
    wheel.is_kinematic = True
    wheel.friction = 0.9
    wheel.set_collision_shape(
        dart.CollisionShape.cylinder(
            _PRECESSION_WHEEL_RADIUS,
            _PRECESSION_WHEEL_HALF_HEIGHT,
        )
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_precession_runtime")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_PRECESSION_GROUND_HALF_EXTENTS)),
        (0.34, 0.43, 0.32),
        name="plan083_precession_ground_visual",
    )
    bridge.add_rigid_body_visual(
        wheel,
        dart.CylinderShape(
            _PRECESSION_WHEEL_RADIUS,
            2.0 * _PRECESSION_WHEEL_HALF_HEIGHT,
        ),
        (0.22, 0.52, 0.78),
        name="plan083_precession_wheel_visual",
    )
    bridge.sync()

    wheel_height_history: deque[float] = deque(maxlen=120)
    spin_rate_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        wheel_position = np.asarray(wheel.translation, dtype=float).reshape(3)
        wheel_velocity = np.asarray(wheel.angular_velocity, dtype=float).reshape(3)
        ground_clearance = float(wheel_position[2] - _PRECESSION_WHEEL_RADIUS)
        spin_rate = float(np.linalg.norm(wheel_velocity))
        wheel_height_history.append(float(wheel_position[2]))
        spin_rate_history.append(spin_rate)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text("rolling wheels: 1")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"wheel height: {wheel_position[2]:.3f} m")
        builder.text(f"wheel ground clearance: {ground_clearance:.4f} m")
        builder.text(f"spin rate: {spin_rate:.3f} rad/s")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if wheel_height_history:
            builder.separator()
            builder.plot_lines("Wheel height", list(wheel_height_history))
            builder.plot_lines("Spin rate", list(spin_rate_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "ground": ground,
            "wheel": wheel,
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _build_ragdoll_runtime(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.World(
        time_step=0.005,
        gravity=(0.0, 0.0, -9.81),
        rigid_body_solver=dart.RigidBodySolver.IPC,
    )

    ground = world.add_rigid_body("plan083_ragdoll_ground", position=(0.0, 0.0, -0.025))
    ground.is_static = True
    ground.friction = 0.8
    ground.set_collision_shape(dart.CollisionShape.box(_RAGDOLL_GROUND_HALF_EXTENTS))

    torso = world.add_rigid_body(
        "plan083_ragdoll_torso",
        position=(0.0, 0.0, 0.42),
        linear_velocity=(0.04, 0.0, -0.04),
        angular_velocity=(0.0, 0.6, 0.25),
    )
    torso.mass = 0.35
    torso.is_kinematic = True
    torso.friction = 0.7
    torso.set_collision_shape(dart.CollisionShape.box(_RAGDOLL_TORSO_HALF_EXTENTS))

    parts = []
    for index, (name, position, shape_kind) in enumerate(_RAGDOLL_PARTS):
        part = world.add_rigid_body(
            f"plan083_ragdoll_{name}",
            position=tuple(position),
            linear_velocity=(0.04, 0.0, -0.04),
            angular_velocity=(0.0, 0.6, 0.25),
        )
        part.mass = 0.08 if shape_kind == "sphere" else 0.12
        part.is_kinematic = True
        part.friction = 0.8
        if shape_kind == "sphere":
            part.set_collision_shape(dart.CollisionShape.sphere(_RAGDOLL_HEAD_RADIUS))
        elif shape_kind == "arm":
            part.set_collision_shape(dart.CollisionShape.box(_RAGDOLL_ARM_HALF_EXTENTS))
        else:
            part.set_collision_shape(dart.CollisionShape.box(_RAGDOLL_LEG_HALF_EXTENTS))
        world.add_rigid_body_revolute_joint(
            f"plan083_ragdoll_joint_{index}",
            torso,
            part,
            axis=(0.0, 0.0, 1.0) if shape_kind == "sphere" else (0.0, 1.0, 0.0),
        )
        parts.append(part)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="plan083_ragdoll_runtime")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_RAGDOLL_GROUND_HALF_EXTENTS)),
        (0.34, 0.43, 0.32),
        name="plan083_ragdoll_ground_visual",
    )
    bridge.add_rigid_body_visual(
        torso,
        dart.BoxShape(_full(_RAGDOLL_TORSO_HALF_EXTENTS)),
        (0.22, 0.42, 0.76),
        name="plan083_ragdoll_torso_visual",
    )
    for part, (name, _, shape_kind) in zip(parts, _RAGDOLL_PARTS):
        if shape_kind == "sphere":
            shape = dart.SphereShape(_RAGDOLL_HEAD_RADIUS)
        elif shape_kind == "arm":
            shape = dart.BoxShape(_full(_RAGDOLL_ARM_HALF_EXTENTS))
        else:
            shape = dart.BoxShape(_full(_RAGDOLL_LEG_HALF_EXTENTS))
        bridge.add_rigid_body_visual(
            part,
            shape,
            (0.72, 0.44, 0.26) if shape_kind != "sphere" else (0.82, 0.64, 0.44),
            name=f"plan083_ragdoll_{name}_visual",
        )
    bridge.sync()

    torso_height_history: deque[float] = deque(maxlen=120)
    leg_clearance_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        torso_position = np.asarray(torso.translation, dtype=float).reshape(3)
        leg_clearances = [
            float(np.asarray(part.translation, dtype=float).reshape(3)[2])
            - _RAGDOLL_LEG_HALF_EXTENTS[2]
            for part, (_, _, shape_kind) in zip(parts, _RAGDOLL_PARTS)
            if shape_kind == "leg"
        ]
        min_clearance = min(leg_clearances)
        torso_height_history.append(float(torso_position[2]))
        leg_clearance_history.append(min_clearance)

        builder.text("status: reduced runtime smoke scene")
        builder.text("solver: rigid IPC World.step")
        builder.text(f"row: {', '.join(target.row_ids)}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"reduced ragdoll bodies: {1 + len(parts)}")
        builder.text(f"revolute joints: {world.num_rigid_body_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"torso height: {torso_position[2]:.3f} m")
        builder.text(f"min leg clearance: {min_clearance:.4f} m")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.text(f"limitation: {target.limitation}")
        builder.separator()
        bridge.build_control_panel(builder, context)
        if torso_height_history:
            builder.separator()
            builder.plot_lines("Torso height", list(torso_height_history))
            builder.plot_lines("Min leg clearance", list(leg_clearance_history))

    info = _target_info(target)
    info.update(
        {
            "sx_world": world,
            "runtime_smoke_scene": True,
            "rigid_body_solver": "ipc",
            "ground": ground,
            "torso": torso,
            "ragdoll_parts": tuple(parts),
        }
    )
    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(target.title, build_panel)],
        info=info,
    )


def _scene(target: Plan083SceneTarget) -> PythonDemoScene:
    if target.scene_id == "plan083_hanging_bridge":
        build = lambda target=target: _build_hanging_bridge_runtime(target)
    elif target.scene_id == "plan083_pulley_system":
        build = lambda target=target: _build_pulley_runtime(target)
    elif target.scene_id == "plan083_umbrella":
        build = lambda target=target: _build_umbrella_runtime(target)
    elif target.scene_id == "plan083_nunchaku":
        build = lambda target=target: _build_nunchaku_runtime(target)
    elif target.scene_id == "plan083_precession":
        build = lambda target=target: _build_precession_runtime(target)
    elif target.scene_id == "plan083_ragdolls":
        build = lambda target=target: _build_ragdoll_runtime(target)
    elif target.scene_id == "plan083_terrain_vehicle":
        build = lambda target=target: _build_terrain_vehicle_runtime(target)
    elif target.scene_id == "plan083_windmill":
        build = lambda target=target: _build_windmill_runtime(target)
    elif target.scene_id == "plan083_lying_flat":
        build = lambda target=target: _build_lying_flat_runtime(target)
    elif target.scene_id == "plan083_candy":
        build = lambda target=target: _build_candy_runtime(target)
    else:
        build = lambda target=target: _build_placeholder(target)

    return PythonDemoScene(
        id=target.scene_id,
        title=target.title,
        category=target.category,
        summary=target.summary,
        build=build,
    )


PLAN083_SCENE_TARGETS: tuple[Plan083SceneTarget, ...] = (
    Plan083SceneTarget(
        scene_id="plan083_lying_flat",
        title="PLAN-083 Lying Flat",
        row_ids=("unb-fig-01",),
        category="PLAN-083 Mixed Corpus",
        summary="Reduced lying-flat cloth and obstacle smoke scene running through World::step.",
        target="Paper Fig. 1 / Table 2 mixed-domain stress scene; reduced to a deformable cloth patch over static rigid obstacle proxies for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_lying_flat --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_lying_flat --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-lying-flat-packet",
        limitation="Reduced deformable-cloth/static-obstacle smoke packet only; rigid rings, deformable tori, rods, articulated ragdoll, cloth self-contact, and paper-scale mixed coupling remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_hanging_bridge",
        title="PLAN-083 Hanging Bridge",
        row_ids=("unb-fig-02",),
        category="PLAN-083 Mixed Corpus",
        summary="Reduced hanging-bridge smoke scene running through World::step.",
        target="Paper Fig. 2 / Table 2 hanging bridge scene; reduced to rigid boards, point connections, and a traveler for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_hanging_bridge --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_hanging_bridge --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-hanging-bridge-packet",
        limitation="Reduced runtime smoke and CPU packet only; waiting for rod, rigid, and codimensional paper-scale coupling.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_pulley_system",
        title="PLAN-083 Pulley System",
        row_ids=("unb-fig-03",),
        category="PLAN-083 Constraints Corpus",
        summary="Reduced pulley constraint smoke scene running through World::step.",
        target="Paper Fig. 3/Fig. 21 pulley force-comparison scene; reduced to a hinged wheel and two point-connected loads for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_pulley_system --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_pulley_system --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-pulley-packet",
        limitation="Reduced hinged-wheel and point-connection smoke packet only; analytical force comparison and paper-scale rope/rod coupling remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_umbrella",
        title="PLAN-083 Umbrella",
        row_ids=("unb-fig-04",),
        category="PLAN-083 Mixed Corpus",
        summary="Reduced umbrella rod-skeleton smoke scene running through World::step.",
        target="Paper Fig. 4 umbrella scene; reduced to a hinged mast, hub, and point-connected ribs for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_umbrella --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_umbrella --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-umbrella-packet",
        limitation="Reduced hinged-rib smoke packet only; cloth shrinking, wrinkling, sliding constraints, and paper-scale rod coupling remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_terrain_vehicle",
        title="PLAN-083 Terrain Vehicle",
        row_ids=("unb-fig-10",),
        category="PLAN-083 Robot Corpus",
        summary="Reduced terrain vehicle smoke scene running through World::step.",
        target="Paper Fig. 10 / Table 2 terrain navigation scene; reduced to a chassis, passive wheel hinges, and flat terrain contact for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_terrain_vehicle --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_terrain_vehicle --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-terrain-vehicle-packet",
        limitation="Reduced chassis/passive-wheel terrain smoke and CPU packet only; terrain mesh, navigation controls, and Table 2 timing remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_ragdolls",
        title="PLAN-083 Ragdolls",
        row_ids=("unb-fig-11",),
        category="PLAN-083 Robot Corpus",
        summary="Reduced six-body ragdoll smoke scene running through World::step.",
        target="Paper Fig. 11 / Table 2 60-ragdoll benchmark scene; reduced to one revolute-chain ragdoll and flat ground contact for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_ragdolls --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_ragdolls --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-ragdoll-packet",
        limitation="Reduced six-body revolute-chain smoke and CPU packet only; cone-twist joints, 60-ragdoll scale, and Table 2 timing remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_nunchaku",
        title="PLAN-083 Nunchaku",
        row_ids=("unb-fig-13", "unb-fig-25"),
        category="PLAN-083 Constraints Corpus",
        summary="Reduced hinge nunchaku smoke scene running through World::step.",
        target="Paper Fig. 13 and Fig. 25 nunchaku rows; reduced to a single rigid hinge for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_nunchaku --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_nunchaku --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-nunchaku-packet",
        limitation="Reduced single-hinge runtime smoke and CPU packet only; cone-twist ranges and N-by-N scaling remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_windmill",
        title="PLAN-083 Windmill",
        row_ids=("unb-fig-20",),
        category="PLAN-083 Constraints Corpus",
        summary="Reduced windmill hinge/contact smoke scene running through World::step.",
        target="Paper Fig. 20 hinge/contact comparison scene; reduced to a hinged blade and falling striker for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_windmill --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_windmill --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-windmill-packet",
        limitation="Reduced hinge/contact smoke and CPU packet only; Bullet/reference comparison and cube piles remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_candy",
        title="PLAN-083 Candy",
        row_ids=("unb-fig-22",),
        category="PLAN-083 Mixed Corpus",
        summary="Reduced deformable candy smoke scene running through World::step.",
        target="Paper Fig. 22 / Table 2 candy mixed-domain scene; reduced to a deformable cloth patch settling against a static shell obstacle for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_candy --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_candy --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-candy-packet",
        limitation="Reduced deformable-cloth/static-shell smoke packet only; affine body packing, twisted shell, and cloth self-contact parity remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_precession",
        title="PLAN-083 Precession",
        row_ids=("unb-fig-23",),
        category="PLAN-083 Robot Corpus",
        summary="Reduced precession wheel smoke scene running through World::step.",
        target="Paper Fig. 23 / Table 2 precession benchmark scene; reduced to a single rolling wheel and flat terrain contact for runtime smoke evidence.",
        smoke_command="pixi run py-demos -- --scene plan083_precession --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_precession --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm-plan083-cpu-precession-packet",
        limitation="Reduced rolling-wheel smoke and CPU packet only; angular-velocity sweep, rolling-contact model, and Table 2 timing remain planned.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_abd_complex_geometry",
        title="PLAN-083 ABD Complex Geometry",
        row_ids=("abd-complex-geometry",),
        category="PLAN-083 ABD Corpus",
        summary="Planned ABD complex-geometry benchmark and py-demo.",
        target="ABD deck complex-geometry contact scene.",
        smoke_command="pixi run py-demos -- --scene plan083_abd_complex_geometry --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_abd_complex_geometry --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=abd_complex_geometry",
        limitation="Waiting for ABD runtime stepping beyond primitive/oracle rows.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_abd_fem_coupling",
        title="PLAN-083 ABD FEM Coupling",
        row_ids=("abd-fem-coupling",),
        category="PLAN-083 ABD Corpus",
        summary="Planned ABD plus FEM deformable coupling scene.",
        target="ABD deck FEM-coupling benchmark and py-demo.",
        smoke_command="pixi run py-demos -- --scene plan083_abd_fem_coupling --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_abd_fem_coupling --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=abd_fem_coupling",
        limitation="Waiting for ABD/FEM mixed runtime coupling.",
    ),
)


_CATEGORY_ORDER = {
    "PLAN-083 Mixed Corpus": 0,
    "PLAN-083 Constraints Corpus": 1,
    "PLAN-083 Robot Corpus": 2,
    "PLAN-083 ABD Corpus": 3,
}


PLAN083_SCENES: tuple[PythonDemoScene, ...] = tuple(
    _scene(target)
    for target in sorted(
        PLAN083_SCENE_TARGETS,
        key=lambda target: (_CATEGORY_ORDER[target.category], target.scene_id),
    )
)
