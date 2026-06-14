"""Renderer fidelity and debug-overlay fixture for the Python demo viewer."""

from __future__ import annotations

import math

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, ScenePanel, SceneSetup


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _rotation_z(angle: float) -> np.ndarray:
    transform = np.eye(4)
    c = math.cos(angle)
    s = math.sin(angle)
    transform[:3, :3] = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    return transform


def _translated_rotation(x: float, y: float, z: float, angle: float) -> np.ndarray:
    transform = _rotation_z(angle)
    transform[:3, 3] = (x, y, z)
    return transform


def _add_frame(
    scene: object,
    name: str,
    shape: object,
    transform: np.ndarray,
    rgba: tuple[float, float, float, float],
    *,
    metallic: float,
    roughness: float,
    reflectance: float | None = None,
) -> object:
    frame = dart.SimpleFrame(dart.gui.world_render_frame(), name, transform)
    frame.set_shape(shape)
    visual = frame.create_visual_aspect()
    visual.set_rgba(np.asarray(rgba, dtype=float))
    visual.set_metallic(float(metallic))
    visual.set_roughness(float(roughness))
    if reflectance is not None:
        visual.set_reflectance(float(reflectance))
    scene.add_simple_frame(frame)
    return frame


def _debug_line(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
    label: str,
) -> object:
    line = dart.gui.DebugLineDescriptor()
    line.from_point = np.asarray(start, dtype=float)
    line.to_point = np.asarray(end, dtype=float)
    line.rgba = np.asarray(rgba, dtype=float)
    line.thickness = 0.018
    line.label = label
    return line


def _make_debug_skeleton() -> tuple[object, object, object, object]:
    skeleton = dart.Skeleton("debug_reference")

    hinge_joint, hinge_body = skeleton.create_revolute_joint_and_body_node_pair()
    hinge_joint.set_name("debug_hinge")
    hinge_joint.set_axis(np.array([0.0, 0.0, 1.0]))
    hinge_joint.set_transform_from_parent_body_node(_translation(0.0, -0.82, 0.72))
    hinge_joint.set_velocity(0, 1.2)

    free_joint, free_body = skeleton.create_free_joint_and_body_node_pair()
    free_joint.set_name("debug_free")
    free_joint.set_transform_from_parent_body_node(_translation(0.0, 0.82, 0.72))
    velocities = np.zeros(6)
    velocities[3] = 1.1
    free_joint.set_velocities(velocities)

    return skeleton, hinge_joint, hinge_body, free_body


def build() -> SceneSetup:
    render_scene = dart.gui.DescriptorRenderScene(
        dart.World(), "gui_fidelity_debug_visuals"
    )
    _add_frame(
        render_scene,
        "matte_ground",
        dart.BoxShape(np.array([3.2, 2.0, 0.04])),
        _translation(0.0, 0.0, -0.04),
        (0.36, 0.39, 0.41, 1.0),
        metallic=0.0,
        roughness=0.82,
        reflectance=0.32,
    )
    _add_frame(
        render_scene,
        "brushed_metal_box",
        dart.BoxShape(np.array([0.44, 0.44, 0.44])),
        _translation(-0.78, -0.18, 0.26),
        (0.84, 0.58, 0.32, 1.0),
        metallic=0.95,
        roughness=0.24,
        reflectance=0.72,
    )
    _add_frame(
        render_scene,
        "rough_ceramic_sphere",
        dart.SphereShape(0.27),
        _translation(0.0, -0.2, 0.29),
        (0.28, 0.53, 0.86, 1.0),
        metallic=0.0,
        roughness=0.9,
        reflectance=0.38,
    )
    glossy = _add_frame(
        render_scene,
        "glossy_debug_bar",
        dart.BoxShape(np.array([0.9, 0.08, 0.08])),
        _translated_rotation(0.82, -0.18, 0.38, 0.22),
        (0.26, 0.78, 0.58, 1.0),
        metallic=0.1,
        roughness=0.18,
        reflectance=0.88,
    )

    _skeleton, hinge_joint, hinge_body, free_body = _make_debug_skeleton()

    state = {
        "phase": 0.0,
        "draw_axes": True,
        "draw_velocities": True,
        "draw_reference": True,
    }

    def pre_step() -> None:
        state["phase"] = float(state["phase"]) + 1.0 / 90.0
        angle = 0.22 + 0.35 * math.sin(float(state["phase"]))
        glossy.set_transform(_translated_rotation(0.82, -0.18, 0.38, angle))
        hinge_joint.set_position(0, angle)
        hinge_joint.set_velocity(0, 1.2 * math.cos(float(state["phase"])))

    def debug_provider() -> object:
        options = dart.gui.DebugDrawOptions()
        options.draw_grid = False
        options.draw_world_frame = False
        options.draw_contacts = False
        options.draw_joint_axes = bool(state["draw_axes"])
        options.draw_linear_velocities = bool(state["draw_velocities"])
        options.draw_angular_velocities = bool(state["draw_velocities"])
        options.joint_axis_length = 0.34
        options.linear_velocity_scale = 0.28
        options.angular_velocity_scale = 0.22
        options.velocity_min_length = 0.04
        options.velocity_max_length = 0.7

        scene = dart.gui.DebugScene()
        lines = []
        if state["draw_reference"]:
            lines.extend(
                [
                    _debug_line(
                        (-1.15, -0.82, 0.72),
                        (1.15, -0.82, 0.72),
                        (0.98, 0.8, 0.2, 1.0),
                        "reference.hinge_row",
                    ),
                    _debug_line(
                        (-1.15, 0.82, 0.72),
                        (1.15, 0.82, 0.72),
                        (0.25, 0.84, 0.95, 1.0),
                        "reference.velocity_row",
                    ),
                ]
            )
        lines.extend(
            dart.gui.make_joint_axis_debug_lines(hinge_body, options, "hinge")
        )
        lines.extend(dart.gui.make_velocity_debug_lines(hinge_body, options, "hinge"))
        lines.extend(dart.gui.make_velocity_debug_lines(free_body, options, "free"))
        scene.lines = lines
        return scene

    def build_panel(builder: object, context: object) -> None:
        builder.text("renderer: descriptor materials")
        builder.text("overlay: debug provider")
        changed, draw_axes = builder.checkbox("Joint axes", bool(state["draw_axes"]))
        if changed:
            state["draw_axes"] = bool(draw_axes)
        changed, draw_velocities = builder.checkbox(
            "Velocities", bool(state["draw_velocities"])
        )
        if changed:
            state["draw_velocities"] = bool(draw_velocities)
        changed, draw_reference = builder.checkbox(
            "Reference rows", bool(state["draw_reference"])
        )
        if changed:
            state["draw_reference"] = bool(draw_reference)
        builder.separator()
        builder.text("metallic box: 0.95 / roughness 0.24")
        builder.text("ceramic sphere: 0.00 / roughness 0.90")
        builder.text("glossy bar: reflectance 0.88")

    return SceneSetup(
        world=render_scene,
        pre_step=pre_step,
        panels=[ScenePanel("Renderer Fidelity", build_panel)],
        debug_provider=debug_provider,
        info={"debug_state": state, "debug_skeleton": _skeleton},
    )


SCENE = PythonDemoScene(
    id="gui_fidelity_debug_visuals",
    title="GUI Fidelity Debug Visuals",
    category="Renderer & Debug",
    summary="Per-shape PBR material contrast plus debug-provider overlay lines.",
    build=build,
)
