"""Pre-contact surrogate contact-gradient diagnostic (World).

This scene is the pre-contact counterpart to ``diff_drone_liftoff``. A rigid
sphere is above the ground and moving downward, so the forward step has no
active contact and both gradient modes produce the same next state. The
backward pass is the point of the scene:

- ``ContactGradientMode.ANALYTIC`` stays exactly contact-free.
- ``ContactGradientMode.PRE_CONTACT_SURROGATE`` adds a backward-only surrogate
  block that anticipates the approaching contact.

The visual view is intentionally compact: two lanes share the same forward
motion and clearance, while the right lane highlights the surrogate sensitivity
change. When DART is built without differentiable support, the scene still
renders the scenario and records finite fallback metrics.
"""

from __future__ import annotations

from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1e-3
_SPHERE_RADIUS = 0.5
_SPHERE_MASS = 2.0
_INITIAL_CENTER_Z = 1.0
_INITIAL_VZ = -0.5
_GROUND_HALF_HEIGHT = 0.5
_GRAVITY_Z = -9.81


def _translation(translation: Any) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def _finite_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if np.isfinite(result) else default


def _freefall_jacobian() -> np.ndarray:
    jacobian = np.zeros((6, 6), dtype=float)
    jacobian[:3, :3] = np.eye(3)
    jacobian[:3, 3:] = _TIME_STEP * np.eye(3)
    jacobian[3:, 3:] = np.eye(3)
    return jacobian


def _build_pre_contact_world(mode: Any) -> Any:
    world = sx.World(
        time_step=_TIME_STEP,
        gravity=(0.0, 0.0, _GRAVITY_Z),
        differentiable=True,
        contact_solver_method=sx.ContactSolverMethod.BOXED_LCP,
        contact_gradient_mode=mode,
    )

    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -_GROUND_HALF_HEIGHT))
    ground.is_static = True
    ground.set_collision_shape(
        sx.CollisionShape.box((5.0, 5.0, _GROUND_HALF_HEIGHT))
    )
    ground.friction = 0.0

    sphere = world.add_rigid_body(
        "sphere", mass=_SPHERE_MASS, position=(0.0, 0.0, _INITIAL_CENTER_Z)
    )
    sphere.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
    sphere.friction = 0.0
    sphere.linear_velocity = (0.0, 0.0, _INITIAL_VZ)

    return world


def _evaluate_mode(mode: Any) -> dict[str, Any]:
    world = _build_pre_contact_world(mode)
    pre_contact_count = len(world.collide())
    initial_state = np.asarray(world.state_vector, dtype=float)

    derivative_error = ""
    jacobian: np.ndarray | None = None
    world.step()
    next_state = np.asarray(world.state_vector, dtype=float)
    post_contact_count = len(world.collide())

    try:
        jacobian = np.asarray(world.get_step_derivatives().state_jacobian, dtype=float)
    except Exception as error:  # noqa: BLE001 - diff support is optional.
        derivative_error = str(error).splitlines()[0] if str(error) else type(error).__name__

    return {
        "initial_state": initial_state,
        "next_state": next_state,
        "jacobian": jacobian,
        "pre_contact_count": pre_contact_count,
        "post_contact_count": post_contact_count,
        "derivative_error": derivative_error,
    }


def _max_abs(values: np.ndarray) -> float:
    if values.size == 0:
        return 0.0
    return float(np.max(np.abs(values)))


def _evaluate_pre_contact_pair() -> dict[str, Any]:
    try:
        analytic = _evaluate_mode(sx.ContactGradientMode.ANALYTIC)
        surrogate = _evaluate_mode(sx.ContactGradientMode.PRE_CONTACT_SURROGATE)
    except Exception as error:  # noqa: BLE001 - scene build must be robust.
        return {
            "status": "fallback",
            "differentiable_available": False,
            "error": str(error).splitlines()[0] if str(error) else type(error).__name__,
        }

    freefall = _freefall_jacobian()
    analytic_jacobian = analytic["jacobian"]
    surrogate_jacobian = surrogate["jacobian"]
    differentiable_available = (
        isinstance(analytic_jacobian, np.ndarray)
        and analytic_jacobian.shape == (6, 6)
        and isinstance(surrogate_jacobian, np.ndarray)
        and surrogate_jacobian.shape == (6, 6)
    )

    forward_state_max_abs_diff = _max_abs(
        np.asarray(analytic["next_state"]) - np.asarray(surrogate["next_state"])
    )
    initial_clearance = _INITIAL_CENTER_Z - _SPHERE_RADIUS
    analytic_post_clearance = (
        float(np.asarray(analytic["next_state"])[2]) - _SPHERE_RADIUS
    )
    surrogate_post_clearance = (
        float(np.asarray(surrogate["next_state"])[2]) - _SPHERE_RADIUS
    )

    if differentiable_available:
        analytic_freefall_error = _max_abs(analytic_jacobian - freefall)
        surrogate_block_magnitude = _max_abs(surrogate_jacobian - freefall)
        analytic_dvzprime_dvz = float(analytic_jacobian[5, 5])
        surrogate_dvzprime_dvz = float(surrogate_jacobian[5, 5])
        surrogate_delta_dvzprime_dvz = float(
            surrogate_jacobian[5, 5] - freefall[5, 5]
        )
        surrogate_delta_dzprime_dvz = float(surrogate_jacobian[2, 5] - freefall[2, 5])
        in_plane_sensitivity_error = abs(float(surrogate_jacobian[3, 3]) - 1.0)
        thresholds_pass = (
            analytic["pre_contact_count"] == 0
            and surrogate["pre_contact_count"] == 0
            and forward_state_max_abs_diff < 1e-12
            and analytic_freefall_error < 1e-9
            and surrogate_block_magnitude > 1e-3
            and surrogate_dvzprime_dvz < 0.5
            and in_plane_sensitivity_error < 1e-9
        )
    else:
        analytic_freefall_error = 0.0
        surrogate_block_magnitude = 0.0
        analytic_dvzprime_dvz = 1.0
        surrogate_dvzprime_dvz = 1.0
        surrogate_delta_dvzprime_dvz = 0.0
        surrogate_delta_dzprime_dvz = 0.0
        in_plane_sensitivity_error = 0.0
        thresholds_pass = False

    return {
        "status": (
            "pre_contact_surrogate"
            if thresholds_pass
            else ("fallback" if not differentiable_available else "threshold_failed")
        ),
        "differentiable_available": differentiable_available,
        "thresholds_pass": thresholds_pass,
        "analytic": analytic,
        "surrogate": surrogate,
        "initial_clearance": initial_clearance,
        "analytic_post_clearance": analytic_post_clearance,
        "surrogate_post_clearance": surrogate_post_clearance,
        "forward_state_max_abs_diff": forward_state_max_abs_diff,
        "analytic_freefall_error": analytic_freefall_error,
        "surrogate_block_magnitude": surrogate_block_magnitude,
        "analytic_dvzprime_dvz": analytic_dvzprime_dvz,
        "surrogate_dvzprime_dvz": surrogate_dvzprime_dvz,
        "surrogate_delta_dvzprime_dvz": surrogate_delta_dvzprime_dvz,
        "surrogate_delta_dzprime_dvz": surrogate_delta_dzprime_dvz,
        "in_plane_sensitivity_error": in_plane_sensitivity_error,
        "error": analytic.get("derivative_error") or surrogate.get("derivative_error"),
    }


def _make_marker(
    render_world: Any,
    name: str,
    shape: Any,
    color: tuple[float, float, float],
    position: Any,
) -> Any:
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), name, _translation(position)
    )
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)
    return frame


def build(panel_title: str = "Diff Pre-Contact Surrogate") -> SceneSetup:
    evaluation = _evaluate_pre_contact_pair()
    analytic_next_z = _finite_float(
        (evaluation.get("analytic") or {}).get("next_state", [0.0, 0.0, _INITIAL_CENTER_Z])[
            2
        ],
        _INITIAL_CENTER_Z + (_INITIAL_VZ + _GRAVITY_Z * _TIME_STEP) * _TIME_STEP,
    )
    surrogate_next_z = _finite_float(
        (evaluation.get("surrogate") or {}).get(
            "next_state", [0.0, 0.0, _INITIAL_CENTER_Z]
        )[2],
        analytic_next_z,
    )

    bridge = WorldRenderBridge(sx.World(), name="diff_pre_contact_surrogate_render")
    render_world = bridge.render_world
    render_world.set_time_step(_TIME_STEP)

    lane_x = {"ANALYTIC": -0.8, "PRE_CONTACT_SURROGATE": 0.8}
    for mode_name, x in lane_x.items():
        _make_marker(
            render_world,
            f"{mode_name.lower()}_ground",
            dart.BoxShape(np.array([1.0, 0.9, 0.06])),
            (0.38, 0.41, 0.44),
            (x, 0.0, -0.03),
        )
        _make_marker(
            render_world,
            f"{mode_name.lower()}_initial_sphere",
            dart.SphereShape(_SPHERE_RADIUS),
            (0.55, 0.58, 0.62),
            (x, 0.0, _INITIAL_CENTER_Z),
        )
        _make_marker(
            render_world,
            f"{mode_name.lower()}_next_sphere",
            dart.SphereShape(_SPHERE_RADIUS * 0.96),
            (0.18, 0.48, 0.86)
            if mode_name == "ANALYTIC"
            else (0.95, 0.58, 0.16),
            (x, 0.0, analytic_next_z if mode_name == "ANALYTIC" else surrogate_next_z),
        )
        _make_marker(
            render_world,
            f"{mode_name.lower()}_clearance",
            dart.BoxShape(np.array([0.03, 0.03, evaluation.get("initial_clearance", 0.5)])),
            (0.18, 0.75, 0.42),
            (x + 0.55, 0.0, evaluation.get("initial_clearance", 0.5) * 0.5),
        )
        _make_marker(
            render_world,
            f"{mode_name.lower()}_velocity_arrow",
            dart.BoxShape(np.array([0.04, 0.04, 0.32])),
            (0.90, 0.18, 0.18),
            (x - 0.45, 0.0, _INITIAL_CENTER_Z - 0.16),
        )

    surrogate_sensitivity = max(
        0.1,
        min(1.0, abs(_finite_float(evaluation.get("surrogate_dvzprime_dvz"), 1.0))),
    )
    _make_marker(
        render_world,
        "analytic_freefall_sensitivity",
        dart.BoxShape(np.array([0.10, 0.10, 0.70])),
        (0.18, 0.65, 0.95),
        (lane_x["ANALYTIC"] + 0.30, 0.0, 1.70),
    )
    _make_marker(
        render_world,
        "surrogate_projected_sensitivity",
        dart.BoxShape(np.array([0.10, 0.10, 0.70 * surrogate_sensitivity])),
        (0.95, 0.55, 0.12),
        (lane_x["PRE_CONTACT_SURROGATE"] + 0.30, 0.0, 1.35 + 0.35 * surrogate_sensitivity),
    )

    panel_state = {"show_details": False}

    def capture_metrics() -> dict[str, Any]:
        analytic = evaluation.get("analytic") or {}
        surrogate = evaluation.get("surrogate") or {}
        return {
            "row": "diff_pre_contact_surrogate",
            "category": "Differentiable",
            "related_source_row": "rigid_contact_solver_compare",
            "solver": "boxed_lcp_pre_contact_surrogate",
            "scope": "pre_contact_backward_only_surrogate",
            "contact_solver_method": "BOXED_LCP",
            "executor": "World.step default",
            "status": evaluation.get("status", "fallback"),
            "differentiable_available": bool(
                evaluation.get("differentiable_available", False)
            ),
            "thresholds_pass": bool(evaluation.get("thresholds_pass", False)),
            "gradient_modes": ["ANALYTIC", "PRE_CONTACT_SURROGATE"],
            "time_step_ms": _TIME_STEP * 1000.0,
            "sphere_radius": _SPHERE_RADIUS,
            "sphere_mass": _SPHERE_MASS,
            "initial_center_z": _INITIAL_CENTER_Z,
            "initial_clearance": _finite_float(evaluation.get("initial_clearance"), 0.5),
            "initial_vz": _INITIAL_VZ,
            "approach_speed": abs(_INITIAL_VZ),
            "pre_step_contact_count": float(analytic.get("pre_contact_count", 0)),
            "post_step_contact_count": float(analytic.get("post_contact_count", 0)),
            "surrogate_pre_step_contact_count": float(
                surrogate.get("pre_contact_count", 0)
            ),
            "surrogate_post_step_contact_count": float(
                surrogate.get("post_contact_count", 0)
            ),
            "analytic_next_z": analytic_next_z,
            "surrogate_next_z": surrogate_next_z,
            "post_step_clearance": _finite_float(
                evaluation.get("analytic_post_clearance"), analytic_next_z - _SPHERE_RADIUS
            ),
            "surrogate_post_step_clearance": _finite_float(
                evaluation.get("surrogate_post_clearance"),
                surrogate_next_z - _SPHERE_RADIUS,
            ),
            "forward_state_max_abs_diff": _finite_float(
                evaluation.get("forward_state_max_abs_diff")
            ),
            "analytic_freefall_error": _finite_float(
                evaluation.get("analytic_freefall_error")
            ),
            "surrogate_block_magnitude": _finite_float(
                evaluation.get("surrogate_block_magnitude")
            ),
            "analytic_dvzprime_dvz": _finite_float(
                evaluation.get("analytic_dvzprime_dvz"), 1.0
            ),
            "surrogate_dvzprime_dvz": _finite_float(
                evaluation.get("surrogate_dvzprime_dvz"), 1.0
            ),
            "surrogate_delta_dvzprime_dvz": _finite_float(
                evaluation.get("surrogate_delta_dvzprime_dvz")
            ),
            "surrogate_delta_dzprime_dvz": _finite_float(
                evaluation.get("surrogate_delta_dzprime_dvz")
            ),
            "in_plane_sensitivity_error": _finite_float(
                evaluation.get("in_plane_sensitivity_error")
            ),
            "modes": {
                "ANALYTIC": {
                    "next_z": analytic_next_z,
                    "pre_contact_count": float(analytic.get("pre_contact_count", 0)),
                    "post_contact_count": float(analytic.get("post_contact_count", 0)),
                    "dvzprime_dvz": _finite_float(
                        evaluation.get("analytic_dvzprime_dvz"), 1.0
                    ),
                },
                "PRE_CONTACT_SURROGATE": {
                    "next_z": surrogate_next_z,
                    "pre_contact_count": float(
                        surrogate.get("pre_contact_count", 0)
                    ),
                    "post_contact_count": float(
                        surrogate.get("post_contact_count", 0)
                    ),
                    "dvzprime_dvz": _finite_float(
                        evaluation.get("surrogate_dvzprime_dvz"), 1.0
                    ),
                    "block_magnitude": _finite_float(
                        evaluation.get("surrogate_block_magnitude")
                    ),
                },
            },
        }

    def build_panel(builder: object, context: object) -> None:
        del context
        metrics = capture_metrics()
        builder.text("forward: identical no-contact step")
        builder.text(f"status: {metrics['status']}")
        builder.text(
            f"clearance: {metrics['initial_clearance']:.3f} m | "
            f"vz: {metrics['initial_vz']:.3f} m/s"
        )
        builder.text(
            f"forward max diff: {metrics['forward_state_max_abs_diff']:.3e}"
        )
        builder.separator()
        builder.text(
            f"ANALYTIC freefall error: {metrics['analytic_freefall_error']:.3e}"
        )
        builder.text(
            "SURROGATE block: "
            f"{metrics['surrogate_block_magnitude']:.3e}"
        )
        builder.text(
            "dvz'/dvz: "
            f"analytic {metrics['analytic_dvzprime_dvz']:.3f} | "
            f"surrogate {metrics['surrogate_dvzprime_dvz']:.3f}"
        )
        builder.plot_lines(
            "vertical sensitivity",
            [
                float(metrics["analytic_dvzprime_dvz"]),
                float(metrics["surrogate_dvzprime_dvz"]),
            ],
        )
        if builder.button("Toggle derivative details"):
            panel_state["show_details"] = not panel_state["show_details"]
        if panel_state["show_details"]:
            builder.text(
                "surrogate delta dvz'/dvz: "
                f"{metrics['surrogate_delta_dvzprime_dvz']:.3e}"
            )
            builder.text(
                "surrogate delta dz'/dvz: "
                f"{metrics['surrogate_delta_dzprime_dvz']:.3e}"
            )
            builder.text(
                "in-plane sensitivity error: "
                f"{metrics['in_plane_sensitivity_error']:.3e}"
            )

    return SceneSetup(
        world=render_world,
        pre_step=lambda: None,
        panels=[ScenePanel(panel_title, build_panel)],
        info={
            "status": evaluation.get("status", "fallback"),
            "differentiable_available": bool(
                evaluation.get("differentiable_available", False)
            ),
            "gradient_modes": ["ANALYTIC", "PRE_CONTACT_SURROGATE"],
            "initial_clearance": _finite_float(evaluation.get("initial_clearance"), 0.5),
            "forward_state_max_abs_diff": _finite_float(
                evaluation.get("forward_state_max_abs_diff")
            ),
            "surrogate_block_magnitude": _finite_float(
                evaluation.get("surrogate_block_magnitude")
            ),
            "note": evaluation.get("error", ""),
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="diff_pre_contact_surrogate",
    title="Differentiable Pre-Contact Surrogate",
    category="Differentiable",
    summary=(
        "Shows PRE_CONTACT_SURROGATE as a backward-only contact-gradient mode: "
        "a sphere approaches the ground without touching, ANALYTIC remains "
        "pure free fall, and the surrogate lane adds a non-zero pre-contact "
        "Jacobian block while preserving the same forward step."
    ),
    build=build,
)
