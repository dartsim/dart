"""Rigid-body contact inspector for the DART 7 World facade."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_HISTORY = 180
_GROUND_HALF = np.array([0.46, 0.30, 0.04])
_PLANE_VISUAL_HALF = np.array([0.46, 0.30, 0.012])
_SPHERE_RADIUS = 0.16
_SPHERE_BOX_TARGET_HALF = np.array([0.18, 0.16, 0.14])
_BOX_HALF = np.array([0.15, 0.12, 0.10])
_TARGET_RADIUS = 0.18
_CAPSULE_RADIUS = 0.10
_CAPSULE_HALF_HEIGHT = 0.24
_CYLINDER_RADIUS = 0.18
_CYLINDER_HALF_HEIGHT = 0.24
_MESH_HALF = 0.40
_MESH_SPHERE_RADIUS = 0.18
_COMPOUND_SPHERE_RADIUS = 0.18
_COMPOUND_BOX_HALF = np.array([0.14, 0.14, 0.14])
_COMPOUND_PROBE_RADIUS = 0.14
_LANE_ORIGINS = {
    "sphere_box": np.array([-1.70, -0.72, 0.0]),
    "box_ground": np.array([0.0, -0.72, 0.0]),
    "capsule_sphere": np.array([1.70, -0.72, 0.0]),
    "cylinder_sphere": np.array([-1.70, 0.78, 0.0]),
    "mesh_sphere": np.array([0.0, 0.78, 0.0]),
    "compound_sphere": np.array([1.70, 0.78, 0.0]),
    "plane_sphere": np.array([-1.70, 2.28, 0.0]),
}
_MESH_VERTICES = [
    (-_MESH_HALF, -_MESH_HALF, 0.0),
    (_MESH_HALF, -_MESH_HALF, 0.0),
    (-_MESH_HALF, _MESH_HALF, 0.0),
    (_MESH_HALF, _MESH_HALF, 0.0),
]
_MESH_TRIANGLES = [(0, 1, 2), (1, 3, 2)]


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _hidden_transform() -> np.ndarray:
    return _translation((0.0, 0.0, -10.0))


@dataclass
class _ContactLane:
    key: str
    label: str
    target: Any
    probe: Any

    @property
    def names(self) -> frozenset[str]:
        return frozenset((self.target.name, self.probe.name))


class _RigidContactInspector:
    def __init__(self) -> None:
        self.penetration = 0.045
        self.pair_index = 0
        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
        self.world.step_profiling_enabled = True

        self.lanes = [
            self._make_sphere_box_lane(),
            self._make_box_ground_lane(),
            self._make_plane_sphere_lane(),
            self._make_capsule_sphere_lane(),
            self._make_cylinder_sphere_lane(),
            self._make_mesh_sphere_lane(),
            self._make_compound_sphere_lane(),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_contact_inspector")
        self._add_visuals()
        self.contact_marker = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            "selected_contact_point",
            _hidden_transform(),
        )
        self.contact_marker.set_shape(dart.SphereShape(0.035))
        self.contact_marker.create_visual_aspect().set_color([0.98, 0.82, 0.18])
        self.bridge.render_world.add_simple_frame(self.contact_marker)

        self._contact_count_history: deque[float] = deque(maxlen=_HISTORY)
        self._max_depth_history: deque[float] = deque(maxlen=_HISTORY)
        self._selected_depth_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, Any] = {}
        self._reset()

    def _make_sphere_box_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["sphere_box"]
        target = self.world.add_rigid_body(
            "sphere_box_target",
            position=tuple(origin + np.array([0.0, 0.0, _SPHERE_BOX_TARGET_HALF[2]])),
        )
        target.is_static = True
        target.set_collision_shape(sx.CollisionShape.box(_SPHERE_BOX_TARGET_HALF))

        probe = self.world.add_rigid_body("sphere_box_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _ContactLane("sphere_box", "Sphere / box", target, probe)

    def _make_box_ground_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["box_ground"]
        ground = self.world.add_rigid_body(
            "box_ground_plate",
            position=tuple(origin + np.array([0.0, 0.0, -_GROUND_HALF[2]])),
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        probe = self.world.add_rigid_body("box_ground_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        return _ContactLane("box_ground", "Box / ground", ground, probe)

    def _make_capsule_sphere_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["capsule_sphere"]
        target = self.world.add_rigid_body(
            "capsule_sphere_target",
            position=tuple(origin + np.array([0.0, 0.0, 0.28])),
        )
        target.is_static = True
        target.set_collision_shape(sx.CollisionShape.sphere(_TARGET_RADIUS))

        probe = self.world.add_rigid_body("capsule_sphere_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(
            sx.CollisionShape.capsule(_CAPSULE_RADIUS, _CAPSULE_HALF_HEIGHT)
        )
        return _ContactLane("capsule_sphere", "Capsule / sphere", target, probe)

    def _make_cylinder_sphere_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["cylinder_sphere"]
        target = self.world.add_rigid_body(
            "cylinder_sphere_target",
            position=tuple(origin + np.array([0.0, 0.0, 0.28])),
        )
        target.is_static = True
        target.set_collision_shape(
            sx.CollisionShape.cylinder(_CYLINDER_RADIUS, _CYLINDER_HALF_HEIGHT)
        )

        probe = self.world.add_rigid_body("cylinder_sphere_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _ContactLane("cylinder_sphere", "Cylinder / sphere", target, probe)

    def _make_mesh_sphere_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["mesh_sphere"]
        target = self.world.add_rigid_body("mesh_sphere_target", position=tuple(origin))
        target.is_static = True
        target.set_collision_shape(sx.CollisionShape.mesh(_MESH_VERTICES, _MESH_TRIANGLES))

        probe = self.world.add_rigid_body("mesh_sphere_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.sphere(_MESH_SPHERE_RADIUS))
        return _ContactLane("mesh_sphere", "Mesh / sphere", target, probe)

    def _make_compound_sphere_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["compound_sphere"]
        target = self.world.add_rigid_body(
            "compound_sphere_target",
            position=tuple(origin + np.array([0.0, 0.0, 0.28])),
        )
        target.is_static = True
        target.add_collision_shape(
            sx.CollisionShape.sphere(
                _COMPOUND_SPHERE_RADIUS,
                _translation((-0.12, 0.0, 0.0)),
            )
        )
        target.add_collision_shape(
            sx.CollisionShape.box(
                _COMPOUND_BOX_HALF,
                _translation((0.20, 0.0, 0.0)),
            )
        )

        probe = self.world.add_rigid_body("compound_sphere_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.sphere(_COMPOUND_PROBE_RADIUS))
        return _ContactLane("compound_sphere", "Compound / sphere", target, probe)

    def _make_plane_sphere_lane(self) -> _ContactLane:
        origin = _LANE_ORIGINS["plane_sphere"]
        target = self.world.add_rigid_body("plane_sphere_target", position=tuple(origin))
        target.is_static = True
        target.set_collision_shape(sx.CollisionShape.plane((0.0, 0.0, 1.0), 0.0))

        probe = self.world.add_rigid_body("plane_sphere_probe")
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _ContactLane("plane_sphere", "Plane / sphere", target, probe)

    def _add_visuals(self) -> None:
        lane_by_key = {lane.key: lane for lane in self.lanes}
        self.bridge.add_rigid_body_visual(
            lane_by_key["sphere_box"].target,
            dart.BoxShape(_full(_SPHERE_BOX_TARGET_HALF)),
            (0.38, 0.40, 0.44),
            name="sphere_box_target_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["box_ground"].target,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.44),
            name="box_ground_plate_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["sphere_box"].probe,
            dart.SphereShape(_SPHERE_RADIUS),
            (0.24, 0.53, 0.88),
            name="sphere_box_probe_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["box_ground"].probe,
            dart.BoxShape(_full(_BOX_HALF)),
            (0.88, 0.42, 0.20),
            name="box_ground_probe_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["capsule_sphere"].target,
            dart.SphereShape(_TARGET_RADIUS),
            (0.40, 0.43, 0.48),
            name="capsule_sphere_target_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["capsule_sphere"].probe,
            dart.CapsuleShape(_CAPSULE_RADIUS, 2.0 * _CAPSULE_HALF_HEIGHT),
            (0.28, 0.68, 0.48),
            name="capsule_sphere_probe_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["cylinder_sphere"].target,
            dart.CylinderShape(_CYLINDER_RADIUS, 2.0 * _CYLINDER_HALF_HEIGHT),
            (0.40, 0.43, 0.48),
            name="cylinder_sphere_target_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["cylinder_sphere"].probe,
            dart.SphereShape(_SPHERE_RADIUS),
            (0.52, 0.40, 0.88),
            name="cylinder_sphere_probe_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["mesh_sphere"].target,
            dart.BoxShape((2.0 * _MESH_HALF, 2.0 * _MESH_HALF, 0.018)),
            (0.38, 0.40, 0.44),
            name="mesh_sphere_target_visual",
            local_transform=_translation((0.0, 0.0, -0.009)),
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["mesh_sphere"].probe,
            dart.SphereShape(_MESH_SPHERE_RADIUS),
            (0.24, 0.53, 0.88),
            name="mesh_sphere_probe_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["compound_sphere"].target,
            dart.SphereShape(_COMPOUND_SPHERE_RADIUS),
            (0.40, 0.43, 0.48),
            name="compound_sphere_target_sphere_visual",
            local_transform=_translation((-0.12, 0.0, 0.0)),
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["compound_sphere"].target,
            dart.BoxShape(_full(_COMPOUND_BOX_HALF)),
            (0.38, 0.40, 0.44),
            name="compound_sphere_target_box_visual",
            local_transform=_translation((0.20, 0.0, 0.0)),
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["compound_sphere"].probe,
            dart.SphereShape(_COMPOUND_PROBE_RADIUS),
            (0.88, 0.42, 0.20),
            name="compound_sphere_probe_visual",
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["plane_sphere"].target,
            dart.BoxShape(_full(_PLANE_VISUAL_HALF)),
            (0.38, 0.40, 0.44),
            name="plane_sphere_target_visual",
            local_transform=_translation((0.0, 0.0, -_PLANE_VISUAL_HALF[2])),
        )
        self.bridge.add_rigid_body_visual(
            lane_by_key["plane_sphere"].probe,
            dart.SphereShape(_SPHERE_RADIUS),
            (0.24, 0.53, 0.88),
            name="plane_sphere_probe_visual",
        )

    def _apply_positions(self) -> None:
        depth = max(0.0, min(0.12, float(self.penetration)))
        self.penetration = depth

        for lane in self.lanes:
            origin = _LANE_ORIGINS[lane.key]
            if lane.key == "sphere_box":
                target = np.asarray(lane.target.translation, dtype=float)
                offset = _SPHERE_BOX_TARGET_HALF[0] + _SPHERE_RADIUS - depth
                lane.probe.transform = _translation(target + np.array([offset, 0.0, 0.0]))
            elif lane.key == "box_ground":
                lane.probe.transform = _translation(
                    origin + np.array([0.0, 0.0, _BOX_HALF[2] - depth])
                )
            elif lane.key == "capsule_sphere":
                target_position = np.asarray(lane.target.translation, dtype=float)
                offset = _TARGET_RADIUS + _CAPSULE_RADIUS - depth
                lane.probe.transform = _translation(
                    target_position + np.array([offset, 0.0, 0.0])
                )
            elif lane.key == "cylinder_sphere":
                target_position = np.asarray(lane.target.translation, dtype=float)
                offset = _CYLINDER_RADIUS + _SPHERE_RADIUS - depth
                lane.probe.transform = _translation(
                    target_position + np.array([offset, 0.0, 0.0])
                )
            elif lane.key == "mesh_sphere":
                lane.probe.transform = _translation(
                    origin + np.array([0.0, 0.0, _MESH_SPHERE_RADIUS - depth])
                )
            elif lane.key == "compound_sphere":
                target_position = np.asarray(lane.target.translation, dtype=float)
                offset = 0.20 + _COMPOUND_BOX_HALF[0] + _COMPOUND_PROBE_RADIUS - depth
                lane.probe.transform = _translation(
                    target_position + np.array([offset, 0.0, 0.0])
                )
            elif lane.key == "plane_sphere":
                lane.probe.transform = _translation(
                    origin + np.array([0.0, 0.0, _SPHERE_RADIUS - depth])
                )
            lane.probe.linear_velocity = (0.0, 0.0, 0.0)
            lane.probe.angular_velocity = (0.0, 0.0, 0.0)
            lane.probe.clear_force()
            lane.probe.clear_torque()

    def _reset(self) -> None:
        self._apply_positions()
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        self._contact_count_history.clear()
        self._max_depth_history.clear()
        self._selected_depth_history.clear()
        self._step_ms_history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def _selected_lane(self) -> _ContactLane:
        index = max(0, min(int(self.pair_index), len(self.lanes) - 1))
        self.pair_index = index
        return self.lanes[index]

    def _contacts(self) -> list[Any]:
        self.world.update_kinematics()
        contacts = list(self.world.collide())
        return sorted(
            contacts,
            key=lambda contact: (
                self._lane_sort_key(contact),
                -float(contact.depth),
                contact.body_a.name,
                contact.body_b.name,
            ),
        )

    def _lane_sort_key(self, contact: Any) -> int:
        names = frozenset((contact.body_a.name, contact.body_b.name))
        for index, lane in enumerate(self.lanes):
            if names == lane.names:
                return index
        return len(self.lanes)

    def _selected_contacts(self, contacts: list[Any]) -> list[Any]:
        selected = self._selected_lane().names
        return [
            contact
            for contact in contacts
            if frozenset((contact.body_a.name, contact.body_b.name)) == selected
        ]

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _record_metrics(self) -> None:
        contacts = self._contacts()
        selected = self._selected_contacts(contacts)
        primary = selected[0] if selected else (contacts[0] if contacts else None)
        depths = [float(contact.depth) for contact in contacts]
        selected_depths = [float(contact.depth) for contact in selected]
        metrics: dict[str, Any] = {
            "total_contact_count": len(contacts),
            "selected_contact_count": len(selected),
            "max_depth": max(depths) if depths else 0.0,
            "selected_max_depth": max(selected_depths) if selected_depths else 0.0,
            "step_ms": self._step_profile_ms(),
            "first_pair": "none",
            "first_depth": 0.0,
            "first_point": np.zeros(3),
            "first_normal": np.zeros(3),
            "first_local_a": np.zeros(3),
            "first_local_b": np.zeros(3),
            "first_shape_indices": (-1, -1),
        }
        if primary is not None:
            metrics.update(
                {
                    "first_pair": f"{primary.body_a.name} -> {primary.body_b.name}",
                    "first_depth": float(primary.depth),
                    "first_point": np.asarray(primary.point, dtype=float),
                    "first_normal": np.asarray(primary.normal, dtype=float),
                    "first_local_a": np.asarray(primary.local_point_a, dtype=float),
                    "first_local_b": np.asarray(primary.local_point_b, dtype=float),
                    "first_shape_indices": (
                        int(primary.shape_index_a),
                        int(primary.shape_index_b),
                    ),
                }
            )
        self._last_metrics = metrics
        self._contact_count_history.append(float(metrics["total_contact_count"]))
        self._max_depth_history.append(float(metrics["max_depth"]))
        self._selected_depth_history.append(float(metrics["selected_max_depth"]))
        self._step_ms_history.append(float(metrics["step_ms"]))
        self._update_marker(metrics)

    def _update_marker(self, metrics: dict[str, Any] | None = None) -> None:
        metrics = self._last_metrics if metrics is None else metrics
        if int(metrics.get("selected_contact_count", 0)) > 0:
            self.contact_marker.set_transform(
                _translation(np.asarray(metrics["first_point"], dtype=float))
            )
        else:
            self.contact_marker.set_transform(_hidden_transform())

    def _sync(self) -> None:
        self.bridge.sync()
        self._update_marker()

    def pre_step(self) -> None:
        self.world.step()
        self._record_metrics()
        self._sync()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "pair_index": int(self.pair_index),
                "penetration": float(self.penetration),
            },
            "contact_count_history": list(self._contact_count_history),
            "max_depth_history": list(self._max_depth_history),
            "selected_depth_history": list(self._selected_depth_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": self._serialize_metrics(self._last_metrics),
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.pair_index = max(
            0,
            min(
                int(controls.get("pair_index", self.pair_index)),
                max(0, len(self.lanes) - 1),
            ),
        )
        self.penetration = float(controls.get("penetration", self.penetration))
        self._contact_count_history.clear()
        self._contact_count_history.extend(
            float(value) for value in state.get("contact_count_history", [])
        )
        self._max_depth_history.clear()
        self._max_depth_history.extend(
            float(value) for value in state.get("max_depth_history", [])
        )
        self._selected_depth_history.clear()
        self._selected_depth_history.extend(
            float(value) for value in state.get("selected_depth_history", [])
        )
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_metrics = self._deserialize_metrics(state.get("last_metrics", {}))
        self._sync()

    def _serialize_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        serialized = dict(metrics)
        for key in ("first_point", "first_normal", "first_local_a", "first_local_b"):
            serialized[key] = np.asarray(metrics.get(key, np.zeros(3)), dtype=float).tolist()
        indices = metrics.get("first_shape_indices", (-1, -1))
        serialized["first_shape_indices"] = [int(indices[0]), int(indices[1])]
        return serialized

    def _deserialize_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        if not metrics:
            return {}
        deserialized = dict(metrics)
        for key in ("first_point", "first_normal", "first_local_a", "first_local_b"):
            deserialized[key] = np.asarray(metrics.get(key, np.zeros(3)), dtype=float)
        indices = metrics.get("first_shape_indices", (-1, -1))
        deserialized["first_shape_indices"] = (int(indices[0]), int(indices[1]))
        return deserialized

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        selected_lane = self._selected_lane()
        contact_values = list(self._contact_count_history)
        depth_values = list(self._max_depth_history)
        selected_depth_values = list(self._selected_depth_history)
        step_values = list(self._step_ms_history)
        metrics = self._serialize_metrics(self._last_metrics)
        shape_indices = metrics.get("first_shape_indices", [-1, -1])
        return {
            "row": "rigid_contact_inspector",
            "solver": "collision_query",
            "executor": "not_applicable_collision_query",
            "contact_scope": "shape_pair_manifold_fields",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "lane_count": int(len(self.lanes)),
            "selected_lane": {
                "key": selected_lane.key,
                "label": selected_lane.label,
                "target": selected_lane.target.name,
                "probe": selected_lane.probe.name,
            },
            "controls": {
                "pair_index": int(self.pair_index),
                "penetration": float(self.penetration),
            },
            "total_contact_count": int(metrics["total_contact_count"]),
            "selected_contact_count": int(metrics["selected_contact_count"]),
            "max_depth": float(metrics["max_depth"]),
            "selected_max_depth": float(metrics["selected_max_depth"]),
            "first_depth": float(metrics["first_depth"]),
            "first_pair": str(metrics["first_pair"]),
            "first_point": metrics["first_point"],
            "first_normal": metrics["first_normal"],
            "first_local_a": metrics["first_local_a"],
            "first_local_b": metrics["first_local_b"],
            "first_shape_indices": [int(shape_indices[0]), int(shape_indices[1])],
            "first_shape_index_a": int(shape_indices[0]),
            "first_shape_index_b": int(shape_indices[1]),
            "metrics": metrics,
            "history": {
                "samples": float(len(contact_values)),
                "max_total_contact_count": max(contact_values, default=0.0),
                "max_depth": max(depth_values, default=0.0),
                "max_selected_depth": max(selected_depth_values, default=0.0),
                "max_step_ms": max(step_values, default=0.0),
            },
        }

    def _format_vec(self, value: Any) -> str:
        vector = np.asarray(value, dtype=float).reshape(3)
        return f"({vector[0]:+.3f}, {vector[1]:+.3f}, {vector[2]:+.3f})"

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [lane.label for lane in self.lanes]
        changed_pair, pair_index = builder.select("Contact pair", self.pair_index, choices)
        if changed_pair:
            self.pair_index = int(pair_index)

        changed_depth, penetration = builder.slider(
            "Penetration", float(self.penetration), 0.0, 0.12
        )
        if changed_depth:
            self.penetration = float(penetration)
            self._reset()
        elif changed_pair:
            self._record_metrics()

        if builder.button("Reset contacts"):
            self._reset()

        if not self._last_metrics:
            self._record_metrics()
        metrics = self._last_metrics
        shape_a, shape_b = metrics["first_shape_indices"]

        builder.separator()
        builder.text("mode: World collision query")
        builder.text(f"selected pair: {self._selected_lane().label}")
        builder.text(
            f"contacts: {int(metrics['selected_contact_count'])} selected / "
            f"{int(metrics['total_contact_count'])} total"
        )
        builder.text(
            f"max depth: {float(metrics['max_depth']):.4f} m | "
            f"selected {float(metrics['selected_max_depth']):.4f} m"
        )
        builder.text(f"first pair: {metrics['first_pair']}")
        builder.text(f"first depth: {float(metrics['first_depth']):.4f} m")
        builder.text(f"point: {self._format_vec(metrics['first_point'])}")
        builder.text(f"normal: {self._format_vec(metrics['first_normal'])}")
        builder.text(f"shape indices: {shape_a} / {shape_b}")
        builder.text(
            f"local A: {self._format_vec(metrics['first_local_a'])} | "
            f"local B: {self._format_vec(metrics['first_local_b'])}"
        )
        builder.text(f"step profile: {float(metrics['step_ms']):.3f} ms")
        builder.plot_lines("Contact count", list(self._contact_count_history))
        builder.plot_lines("Max depth", list(self._max_depth_history))
        builder.plot_lines("Selected depth", list(self._selected_depth_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    inspector = _RigidContactInspector()
    return SceneSetup(
        world=inspector.bridge.render_world,
        pre_step=inspector.pre_step,
        force_drag=inspector.bridge.force_drag,
        renderable_provider=inspector.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Contact Inspector", inspector.build_panel)],
        info={
            "sx_world": inspector.world,
            "rigid_contact_inspector_controller": inspector,
            CAPTURE_METRICS_INFO_KEY: inspector.capture_metrics,
            "replay_capture_state": inspector.capture_replay_state,
            "replay_restore_state": inspector.restore_replay_state,
            "replay_sync": inspector._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_contact_inspector",
    title="Rigid Contact Inspector",
    category="World Rigid Body",
    summary=(
        "Shows rigid-body contact pairs with raw point, normal, depth, and "
        "shape-index diagnostics."
    ),
    build=build,
)
