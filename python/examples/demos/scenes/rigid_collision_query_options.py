"""Rigid-body collision-query option inspector for the DART 7 World facade."""

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
_SPHERE_RADIUS = 0.14
_PENETRATION = 0.06
_CENTER_SPACING = 2.0 * _SPHERE_RADIUS - _PENETRATION
_LANE_ORIGINS = {
    "rigid_rigid": np.array([-1.65, 0.0, 0.0]),
    "rigid_link": np.array([-0.55, 0.0, 0.0]),
    "same_links": np.array([0.55, 0.0, 0.0]),
    "cross_links": np.array([1.65, 0.0, 0.0]),
}


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _hidden_transform() -> np.ndarray:
    return _translation((0.0, 0.0, -10.0))


def _lane_centers(key: str) -> tuple[np.ndarray, np.ndarray]:
    origin = _LANE_ORIGINS[key]
    offset = np.array([0.5 * _CENTER_SPACING, 0.0, 0.0])
    center = origin + np.array([0.0, 0.0, _SPHERE_RADIUS])
    return center - offset, center + offset


@dataclass(frozen=True)
class _QueryLane:
    key: str
    label: str
    body_a: Any
    body_b: Any
    kind: str
    option_key: str

    @property
    def names(self) -> frozenset[str]:
        return frozenset((self.body_a.name, self.body_b.name))


class _RigidCollisionQueryOptions:
    def __init__(self) -> None:
        self.include_rigid_body_pairs = True
        self.include_rigid_body_link_pairs = True
        self.include_link_pairs = True
        self.include_same_multibody_link_pairs = True
        self.ignored_pair_key = "none"

        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_rigid_rigid_lane(),
            self._make_rigid_link_lane(),
            self._make_same_links_lane(),
            self._make_cross_links_lane(),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_collision_query_options")
        self._add_visuals()
        self.markers: dict[str, Any] = {}
        for lane in self.lanes:
            marker = dart.SimpleFrame(
                dart.gui.world_render_frame(),
                f"{lane.key}_active_contact_marker",
                _hidden_transform(),
            )
            marker.set_shape(dart.SphereShape(0.028))
            marker.create_visual_aspect().set_color([0.98, 0.82, 0.18])
            self.bridge.render_world.add_simple_frame(marker)
            self.markers[lane.key] = marker

        self._active_count_history: deque[float] = deque(maxlen=_HISTORY)
        self._filtered_count_history: deque[float] = deque(maxlen=_HISTORY)
        self._ignored_count_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, Any] = {}
        self._record_metrics()
        self._sync()

    def _make_rigid_rigid_lane(self) -> _QueryLane:
        left, right = _lane_centers("rigid_rigid")
        target = self.world.add_rigid_body("query_rigid_target", position=tuple(left))
        target.is_static = True
        target.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))

        probe = self.world.add_rigid_body("query_rigid_probe", position=tuple(right))
        probe.is_kinematic = True
        probe.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _QueryLane(
            "rigid_rigid",
            "Rigid / rigid",
            target,
            probe,
            "rigid body pair",
            "include_rigid_body_pairs",
        )

    def _make_rigid_link_lane(self) -> _QueryLane:
        left, right = _lane_centers("rigid_link")
        body = self.world.add_rigid_body("query_rigid_link_body", position=tuple(left))
        body.is_static = True
        body.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))

        robot = self.world.add_multibody("query_rigid_link_multibody")
        base = robot.add_link("query_rigid_link_base")
        link = robot.add_link(
            "query_rigid_link",
            parent=base,
            joint=sx.JointSpec(
                name="query_rigid_link_fixed",
                type=sx.JointType.FIXED,
                transform_from_parent=_translation(right),
            ),
        )
        link.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _QueryLane(
            "rigid_link",
            "Rigid / link",
            body,
            link,
            "rigid body / link pair",
            "include_rigid_body_link_pairs",
        )

    def _make_same_links_lane(self) -> _QueryLane:
        left, right = _lane_centers("same_links")
        robot = self.world.add_multibody("query_same_link_multibody")
        base = robot.add_link("query_same_link_base")
        link_a = robot.add_link(
            "query_same_link_a",
            parent=base,
            joint=sx.JointSpec(
                name="query_same_link_a_fixed",
                type=sx.JointType.FIXED,
                transform_from_parent=_translation(left),
            ),
        )
        link_a.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        link_b = robot.add_link(
            "query_same_link_b",
            parent=base,
            joint=sx.JointSpec(
                name="query_same_link_b_fixed",
                type=sx.JointType.FIXED,
                transform_from_parent=_translation(right),
            ),
        )
        link_b.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _QueryLane(
            "same_links",
            "Same-multibody links",
            link_a,
            link_b,
            "link pair, same multibody",
            "include_same_multibody_link_pairs",
        )

    def _make_cross_links_lane(self) -> _QueryLane:
        left, right = _lane_centers("cross_links")
        robot_a = self.world.add_multibody("query_cross_link_multibody_a")
        base_a = robot_a.add_link("query_cross_link_base_a")
        link_a = robot_a.add_link(
            "query_cross_link_a",
            parent=base_a,
            joint=sx.JointSpec(
                name="query_cross_link_a_fixed",
                type=sx.JointType.FIXED,
                transform_from_parent=_translation(left),
            ),
        )
        link_a.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))

        robot_b = self.world.add_multibody("query_cross_link_multibody_b")
        base_b = robot_b.add_link("query_cross_link_base_b")
        link_b = robot_b.add_link(
            "query_cross_link_b",
            parent=base_b,
            joint=sx.JointSpec(
                name="query_cross_link_b_fixed",
                type=sx.JointType.FIXED,
                transform_from_parent=_translation(right),
            ),
        )
        link_b.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        return _QueryLane(
            "cross_links",
            "Cross-multibody links",
            link_a,
            link_b,
            "link pair, different multibodies",
            "include_link_pairs",
        )

    def _add_visuals(self) -> None:
        colors = {
            "rigid_rigid": ((0.28, 0.58, 0.92), (0.86, 0.43, 0.22)),
            "rigid_link": ((0.34, 0.56, 0.82), (0.30, 0.72, 0.48)),
            "same_links": ((0.42, 0.48, 0.86), (0.54, 0.38, 0.82)),
            "cross_links": ((0.28, 0.66, 0.70), (0.62, 0.52, 0.24)),
        }
        for lane in self.lanes:
            color_a, color_b = colors[lane.key]
            if isinstance(lane.body_a, sx.RigidBody):
                self.bridge.add_rigid_body_visual(
                    lane.body_a,
                    dart.SphereShape(_SPHERE_RADIUS),
                    color_a,
                    name=f"{lane.key}_a_visual",
                )
            else:
                self.bridge.add_link_visual(
                    lane.body_a,
                    dart.SphereShape(_SPHERE_RADIUS),
                    color_a,
                    name=f"{lane.key}_a_visual",
                )
            if isinstance(lane.body_b, sx.RigidBody):
                self.bridge.add_rigid_body_visual(
                    lane.body_b,
                    dart.SphereShape(_SPHERE_RADIUS),
                    color_b,
                    name=f"{lane.key}_b_visual",
                )
            else:
                self.bridge.add_link_visual(
                    lane.body_b,
                    dart.SphereShape(_SPHERE_RADIUS),
                    color_b,
                    name=f"{lane.key}_b_visual",
                )

    def _options(self, all_pairs: bool = False) -> Any:
        if all_pairs:
            return sx.CollisionQueryOptions()
        return sx.CollisionQueryOptions(
            include_same_multibody_link_pairs=self.include_same_multibody_link_pairs,
            include_rigid_body_pairs=self.include_rigid_body_pairs,
            include_rigid_body_link_pairs=self.include_rigid_body_link_pairs,
            include_link_pairs=self.include_link_pairs,
        )

    def _ignored_pair_choices(self) -> tuple[str, ...]:
        return ("None",) + tuple(lane.label for lane in self.lanes)

    def _ignored_pair_index(self) -> int:
        if self.ignored_pair_key == "none":
            return 0
        for index, lane in enumerate(self.lanes, start=1):
            if lane.key == self.ignored_pair_key:
                return index
        return 0

    def _set_ignored_pair_from_index(self, index: int) -> None:
        if index <= 0:
            self.ignored_pair_key = "none"
        else:
            lanes = self.lanes
            self.ignored_pair_key = lanes[min(index - 1, len(lanes) - 1)].key
        self._record_metrics()

    def _sync_ignored_pairs(self) -> None:
        self.world.clear_ignored_collision_pairs()
        if self.ignored_pair_key == "none":
            return
        for lane in self.lanes:
            if lane.key == self.ignored_pair_key:
                self.world.set_collision_pair_ignored(lane.body_a, lane.body_b)
                return

    def _lane_sort_key(self, contact: Any) -> int:
        names = frozenset((contact.body_a.name, contact.body_b.name))
        for index, lane in enumerate(self.lanes):
            if names == lane.names:
                return index
        return len(self.lanes)

    def _contacts(self, *, all_pairs: bool = False) -> list[Any]:
        self.world.update_kinematics()
        contacts = list(self.world.collide(self._options(all_pairs=all_pairs)))
        return sorted(
            contacts,
            key=lambda contact: (
                self._lane_sort_key(contact),
                contact.body_a.name,
                contact.body_b.name,
            ),
        )

    def _contacts_by_lane(self, contacts: list[Any]) -> dict[str, list[Any]]:
        by_lane = {lane.key: [] for lane in self.lanes}
        for contact in contacts:
            names = frozenset((contact.body_a.name, contact.body_b.name))
            for lane in self.lanes:
                if names == lane.names:
                    by_lane[lane.key].append(contact)
                    break
        return by_lane

    def _collision_body_metrics(self, body: Any) -> dict[str, Any]:
        kind = "rigid" if body.is_rigid_body else "link" if body.is_link else "unknown"
        return {
            "name": body.name,
            "kind": kind,
            "valid": bool(body.is_valid),
            "rigid_cast": body.as_rigid_body() is not None,
            "link_cast": body.as_link() is not None,
        }

    def _contact_body_metrics(self, contact: Any | None) -> list[dict[str, Any]]:
        if contact is None:
            return []
        return [
            self._collision_body_metrics(contact.body_a),
            self._collision_body_metrics(contact.body_b),
        ]

    def _record_metrics(self) -> None:
        self.world.clear_ignored_collision_pairs()
        baseline = self._contacts(all_pairs=True)
        option_filtered = self._contacts()
        self._sync_ignored_pairs()
        active = self._contacts()
        baseline_by_lane = self._contacts_by_lane(baseline)
        option_by_lane = self._contacts_by_lane(option_filtered)
        active_by_lane = self._contacts_by_lane(active)
        lane_metrics: dict[str, dict[str, Any]] = {}
        for lane in self.lanes:
            active_contacts = active_by_lane[lane.key]
            option_contacts = option_by_lane[lane.key]
            baseline_contacts = baseline_by_lane[lane.key]
            first = active_contacts[0] if active_contacts else None
            body_metrics = self._contact_body_metrics(first)
            body_kinds = tuple(str(body["kind"]) for body in body_metrics)
            body_valid = tuple(bool(body["valid"]) for body in body_metrics)
            lane_metrics[lane.key] = {
                "label": lane.label,
                "kind": lane.kind,
                "option_key": lane.option_key,
                "baseline_count": len(baseline_contacts),
                "option_count": len(option_contacts),
                "active_count": len(active_contacts),
                "filtered": len(baseline_contacts) > 0 and not active_contacts,
                "option_filtered": (
                    len(baseline_contacts) > 0 and not option_contacts
                ),
                "pair_ignored": len(option_contacts) > 0 and not active_contacts,
                "first_depth": float(first.depth) if first is not None else 0.0,
                "first_point": (
                    np.asarray(first.point, dtype=float) if first is not None else np.zeros(3)
                ),
                "first_shape_indices": (
                    (int(first.shape_index_a), int(first.shape_index_b))
                    if first is not None
                    else (-1, -1)
                ),
                "first_bodies": body_metrics,
                "first_body_kinds": body_kinds,
                "first_body_valid": body_valid,
                "first_body_pair": (
                    " -> ".join(body_kinds) if body_kinds else "none"
                ),
            }
        active_count = sum(
            int(metrics["active_count"]) for metrics in lane_metrics.values()
        )
        baseline_count = sum(
            int(metrics["baseline_count"]) for metrics in lane_metrics.values()
        )
        option_count = sum(
            int(metrics["option_count"]) for metrics in lane_metrics.values()
        )
        self._last_metrics = {
            "baseline_contact_count": baseline_count,
            "option_contact_count": option_count,
            "active_contact_count": active_count,
            "filtered_contact_count": baseline_count - active_count,
            "option_filtered_contact_count": baseline_count - option_count,
            "ignored_contact_count": option_count - active_count,
            "ignored_pair_key": self.ignored_pair_key,
            "ignored_pair_count": self.world.num_ignored_collision_pairs,
            "lanes": lane_metrics,
        }
        self._active_count_history.append(float(active_count))
        self._filtered_count_history.append(float(baseline_count - active_count))
        self._ignored_count_history.append(float(option_count - active_count))
        self._update_markers()

    def _update_markers(self) -> None:
        if not self._last_metrics:
            return
        for lane in self.lanes:
            metrics = self._last_metrics["lanes"][lane.key]
            if int(metrics["active_count"]) > 0:
                self.markers[lane.key].set_transform(
                    _translation(np.asarray(metrics["first_point"], dtype=float))
                )
            else:
                self.markers[lane.key].set_transform(_hidden_transform())

    def _sync(self) -> None:
        self.bridge.sync()
        self._update_markers()

    def pre_step(self) -> None:
        self.world.step()
        self._record_metrics()
        self._sync()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "include_rigid_body_pairs": bool(self.include_rigid_body_pairs),
                "include_rigid_body_link_pairs": bool(
                    self.include_rigid_body_link_pairs
                ),
                "include_link_pairs": bool(self.include_link_pairs),
                "include_same_multibody_link_pairs": bool(
                    self.include_same_multibody_link_pairs
                ),
                "ignored_pair_key": self.ignored_pair_key,
            },
            "active_count_history": list(self._active_count_history),
            "filtered_count_history": list(self._filtered_count_history),
            "ignored_count_history": list(self._ignored_count_history),
            "last_metrics": self._serialize_metrics(self._last_metrics),
        }

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        metrics = self._serialize_metrics(self._last_metrics)
        return {
            "active_contact_count": int(metrics["active_contact_count"]),
            "baseline_contact_count": int(metrics["baseline_contact_count"]),
            "filtered_contact_count": int(metrics["filtered_contact_count"]),
            "ignored_contact_count": int(metrics["ignored_contact_count"]),
            "ignored_pair_count": int(metrics["ignored_pair_count"]),
            "ignored_pair_key": str(metrics["ignored_pair_key"]),
            "include_link_pairs": bool(self.include_link_pairs),
            "include_rigid_body_link_pairs": bool(
                self.include_rigid_body_link_pairs
            ),
            "include_rigid_body_pairs": bool(self.include_rigid_body_pairs),
            "include_same_multibody_link_pairs": bool(
                self.include_same_multibody_link_pairs
            ),
            "lanes": metrics["lanes"],
            "option_contact_count": int(metrics["option_contact_count"]),
            "option_filtered_contact_count": int(
                metrics["option_filtered_contact_count"]
            ),
            "row": "rigid_collision_query_options",
            "solver": "collision_query",
            "executor": "not_applicable_collision_query",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.include_rigid_body_pairs = bool(
            controls.get("include_rigid_body_pairs", self.include_rigid_body_pairs)
        )
        self.include_rigid_body_link_pairs = bool(
            controls.get(
                "include_rigid_body_link_pairs",
                self.include_rigid_body_link_pairs,
            )
        )
        self.include_link_pairs = bool(
            controls.get("include_link_pairs", self.include_link_pairs)
        )
        self.include_same_multibody_link_pairs = bool(
            controls.get(
                "include_same_multibody_link_pairs",
                self.include_same_multibody_link_pairs,
            )
        )
        self.ignored_pair_key = str(controls.get("ignored_pair_key", self.ignored_pair_key))
        self._active_count_history.clear()
        self._active_count_history.extend(
            float(value) for value in state.get("active_count_history", [])
        )
        self._filtered_count_history.clear()
        self._filtered_count_history.extend(
            float(value) for value in state.get("filtered_count_history", [])
        )
        self._ignored_count_history.clear()
        self._ignored_count_history.extend(
            float(value) for value in state.get("ignored_count_history", [])
        )
        self._last_metrics = self._deserialize_metrics(state.get("last_metrics", {}))
        if not self._last_metrics:
            self._record_metrics()
        else:
            self._sync_ignored_pairs()
        self._sync()

    def _serialize_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        if not metrics:
            return {}
        serialized = dict(metrics)
        lanes: dict[str, dict[str, Any]] = {}
        for key, lane_metrics in metrics.get("lanes", {}).items():
            lane = dict(lane_metrics)
            lane["first_point"] = np.asarray(
                lane.get("first_point", np.zeros(3)), dtype=float
            ).tolist()
            indices = lane.get("first_shape_indices", (-1, -1))
            lane["first_shape_indices"] = [int(indices[0]), int(indices[1])]
            lane["first_bodies"] = [
                dict(body) for body in lane.get("first_bodies", [])
            ]
            lane["first_body_kinds"] = list(lane.get("first_body_kinds", []))
            lane["first_body_valid"] = [
                bool(value) for value in lane.get("first_body_valid", [])
            ]
            lanes[key] = lane
        serialized["lanes"] = lanes
        return serialized

    def _deserialize_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        if not metrics:
            return {}
        deserialized = dict(metrics)
        lanes: dict[str, dict[str, Any]] = {}
        for key, lane_metrics in metrics.get("lanes", {}).items():
            lane = dict(lane_metrics)
            lane["first_point"] = np.asarray(
                lane.get("first_point", np.zeros(3)), dtype=float
            )
            indices = lane.get("first_shape_indices", (-1, -1))
            lane["first_shape_indices"] = (int(indices[0]), int(indices[1]))
            lane["first_bodies"] = [
                dict(body) for body in lane.get("first_bodies", [])
            ]
            lane["first_body_kinds"] = tuple(lane.get("first_body_kinds", ()))
            lane["first_body_valid"] = tuple(
                bool(value) for value in lane.get("first_body_valid", ())
            )
            lanes[key] = lane
        deserialized["lanes"] = lanes
        return deserialized

    def _apply_preset(self, preset: str) -> None:
        if preset == "all":
            self.include_rigid_body_pairs = True
            self.include_rigid_body_link_pairs = True
            self.include_link_pairs = True
            self.include_same_multibody_link_pairs = True
        elif preset == "body_only":
            self.include_rigid_body_pairs = True
            self.include_rigid_body_link_pairs = False
            self.include_link_pairs = False
            self.include_same_multibody_link_pairs = False
        elif preset == "link_only":
            self.include_rigid_body_pairs = False
            self.include_rigid_body_link_pairs = False
            self.include_link_pairs = True
            self.include_same_multibody_link_pairs = True
        elif preset == "no_same":
            self.include_rigid_body_pairs = True
            self.include_rigid_body_link_pairs = True
            self.include_link_pairs = True
            self.include_same_multibody_link_pairs = False
        elif preset == "clear_ignore":
            self.ignored_pair_key = "none"
        elif preset == "ignore_rigid_link":
            self.include_rigid_body_pairs = True
            self.include_rigid_body_link_pairs = True
            self.include_link_pairs = True
            self.include_same_multibody_link_pairs = True
            self.ignored_pair_key = "rigid_link"
        self._record_metrics()

    def _checkbox(self, builder: Any, label: str, attr: str) -> bool:
        changed, value = builder.checkbox(label, bool(getattr(self, attr)))
        if changed:
            setattr(self, attr, bool(value))
        return bool(changed)

    def build_panel(self, builder: Any, context: Any) -> None:
        changed = False
        changed |= self._checkbox(builder, "Rigid body pairs", "include_rigid_body_pairs")
        changed |= self._checkbox(
            builder,
            "Rigid body / link pairs",
            "include_rigid_body_link_pairs",
        )
        changed |= self._checkbox(builder, "Link pairs", "include_link_pairs")
        changed |= self._checkbox(
            builder,
            "Same-multibody link pairs",
            "include_same_multibody_link_pairs",
        )
        changed_ignore, ignored_index = builder.select(
            "Ignored pair",
            self._ignored_pair_index(),
            self._ignored_pair_choices(),
        )

        if builder.button("All query pairs"):
            self._apply_preset("all")
        elif builder.button("Body-only query"):
            self._apply_preset("body_only")
        elif builder.button("Link-only query"):
            self._apply_preset("link_only")
        elif builder.button("No same-multibody"):
            self._apply_preset("no_same")
        elif builder.button("Ignore rigid/link pair"):
            self._apply_preset("ignore_rigid_link")
        elif builder.button("Clear ignored pair"):
            self._apply_preset("clear_ignore")
        elif changed_ignore:
            self._set_ignored_pair_from_index(int(ignored_index))
        elif changed:
            self._record_metrics()

        if not self._last_metrics:
            self._record_metrics()

        metrics = self._last_metrics
        builder.separator()
        builder.text("mode: World.collide(options)")
        builder.text(
            f"contacts: {int(metrics['active_contact_count'])} active / "
            f"{int(metrics['baseline_contact_count'])} baseline"
        )
        builder.text(
            f"option-filtered: {int(metrics['option_filtered_contact_count'])} | "
            f"pair-ignored: {int(metrics['ignored_contact_count'])}"
        )
        builder.text(
            f"ignored pairs: {int(metrics['ignored_pair_count'])} "
            f"({metrics['ignored_pair_key']})"
        )
        for lane in self.lanes:
            lane_metrics = metrics["lanes"][lane.key]
            if int(lane_metrics["active_count"]) > 0:
                status = "active"
            elif bool(lane_metrics["pair_ignored"]):
                status = "pair ignored"
            else:
                status = "option filtered"
            shape_a, shape_b = lane_metrics["first_shape_indices"]
            builder.text(
                f"{lane.label}: {status} "
                f"({int(lane_metrics['active_count'])}/"
                f"{int(lane_metrics['baseline_count'])})"
            )
            builder.text(f"  kind: {lane.kind}")
            if int(lane_metrics["active_count"]) > 0:
                body_metrics = lane_metrics["first_bodies"]
                cast_summary = " / ".join(
                    (
                        f"{body['name']}="
                        f"{'rigid' if body['rigid_cast'] else 'link' if body['link_cast'] else 'unknown'}"
                    )
                    for body in body_metrics
                )
                builder.text(
                    f"  depth: {float(lane_metrics['first_depth']):.4f} m | "
                    f"shape indices: {shape_a} / {shape_b}"
                )
                builder.text(f"  contact bodies: {lane_metrics['first_body_pair']}")
                builder.text(f"  casts: {cast_summary}")
            else:
                builder.text(
                    f"  option: {lane.option_key} | "
                    f"option contacts: {int(lane_metrics['option_count'])}"
                )
        builder.plot_lines("Active contacts", list(self._active_count_history))
        builder.plot_lines("Filtered contacts", list(self._filtered_count_history))
        builder.plot_lines("Pair-ignored contacts", list(self._ignored_count_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    query = _RigidCollisionQueryOptions()
    return SceneSetup(
        world=query.bridge.render_world,
        pre_step=query.pre_step,
        force_drag=query.bridge.force_drag,
        renderable_provider=query.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Collision Query Options", query.build_panel)],
        info={
            "sx_world": query.world,
            "rigid_collision_query_options_controller": query,
            "replay_capture_state": query.capture_replay_state,
            "replay_restore_state": query.restore_replay_state,
            "replay_sync": query._sync,
            CAPTURE_METRICS_INFO_KEY: query.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_collision_query_options",
    title="Rigid Collision Query Options",
    category="World Rigid Body",
    summary=(
        "Shows how World.collide options and ignored pairs include or filter "
        "rigid-body, link, and same-multibody contact pairs."
    ),
    build=build,
)
