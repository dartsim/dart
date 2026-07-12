#!/usr/bin/env python3
"""Record DART 7 World trajectories and contact traces for agent verification."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable

import numpy as np

SCHEMA_VERSION = "dart.trajectory/v0"
CONTACT_SCHEMA_VERSION = "dart.contact_events/v0"
TRAJECTORY_COLUMNS = (
    "frame",
    "time",
    "body",
    "pos_x",
    "pos_y",
    "pos_z",
    "lin_x",
    "lin_y",
    "lin_z",
    "ang_x",
    "ang_y",
    "ang_z",
    "contact_count",
)
_UNITS = {
    "length": "meter",
    "mass": "kilogram",
    "time": "second",
    "angle": "radian",
}


@dataclass
class WorldRunner:
    scene_id: str
    world: Any
    step_once: Callable[[], None]


@dataclass(frozen=True)
class TrackedBody:
    label: str
    body: Any


def _import_dartpy() -> Any:
    import dartpy as dart

    return dart


def _fmt(value: float) -> str:
    return f"{float(value):.17e}"


def _vector(value: Any, length: int = 3) -> list[float]:
    return np.asarray(value, dtype=float).reshape(length).astype(float).tolist()


def _json_compact(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _ensure_simple_label(label: str) -> str:
    if any(char.isspace() for char in label):
        raise ValueError(
            f"body label {label!r} contains whitespace; pass stable no-space labels"
        )
    return label


def _make_free_fall() -> Any:
    sx = _import_dartpy()
    world = sx.World(time_step=0.01, gravity=(0.0, 0.0, -9.81))
    sphere = world.add_rigid_body("faller", mass=1.0, position=(0.0, 0.0, 5.0))
    sphere.set_collision_shape(sx.CollisionShape.sphere(0.25))
    return world


def _make_box_on_ground() -> Any:
    sx = _import_dartpy()
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, -9.81))
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.1))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((2.0, 2.0, 0.1)))
    box = world.add_rigid_body("box", mass=1.0, position=(0.0, 0.0, 0.75))
    box.set_collision_shape(sx.CollisionShape.box((0.1, 0.1, 0.1)))
    return world


def _make_two_body_contact() -> Any:
    sx = _import_dartpy()
    world = sx.World(time_step=0.002, gravity=(0.0, 0.0, -9.81))
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.1))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((1.0, 1.0, 0.1)))
    box = world.add_rigid_body(
        "box",
        mass=1.0,
        position=(0.0, 0.0, 0.095),
        linear_velocity=(0.0, 0.0, -0.1),
    )
    box.set_collision_shape(sx.CollisionShape.box((0.1, 0.1, 0.1)))
    return world


def _make_box_stack() -> Any:
    """Ground plus three resting boxes of decreasing size.

    Mirrors the DART 6 stack evidence scene: a static ground and three stacked
    boxes (bottom, middle, top) that start in contact so the contacts,
    body-frame, and label overlays all have something to draw. The bridge gives
    the dynamic bodies distinct colors by creation order.
    """

    sx = _import_dartpy()
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, -9.81))
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, -0.05))
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box((2.0, 2.0, 0.1)))

    # Ground top sits at z = 0; each box rests on the one below it.
    bottom = world.add_rigid_body("box_bottom", mass=1.0, position=(0.0, 0.0, 0.2))
    bottom.set_collision_shape(sx.CollisionShape.box((0.4, 0.4, 0.4)))
    middle = world.add_rigid_body("box_middle", mass=0.6, position=(0.0, 0.0, 0.55))
    middle.set_collision_shape(sx.CollisionShape.box((0.3, 0.3, 0.3)))
    top = world.add_rigid_body("box_top", mass=0.3, position=(0.0, 0.0, 0.8))
    top.set_collision_shape(sx.CollisionShape.box((0.2, 0.2, 0.2)))
    return world


_BUILTIN_SCENES: dict[str, Callable[[], Any]] = {
    "free_fall": _make_free_fall,
    "box_on_ground": _make_box_on_ground,
    "two_body_contact": _make_two_body_contact,
    "box_stack": _make_box_stack,
}


def _load_factory(spec: str) -> Callable[[], Any]:
    if ":" not in spec:
        raise ValueError("--factory must be in module:callable form")
    module_name, attr_path = spec.split(":", 1)
    target: Any = importlib.import_module(module_name)
    for name in attr_path.split("."):
        target = getattr(target, name)
    if not callable(target):
        raise TypeError(f"{spec!r} did not resolve to a callable")
    return target


def _demo_scene_factory(scene_id: str) -> Callable[[], Any]:
    def build_demo() -> Any:
        from examples.demos.registry import make_demo_scenes

        for scene in make_demo_scenes():
            if scene.id == scene_id:
                return scene.build()
        raise ValueError(f"unknown built-in or demo scene {scene_id!r}")

    return build_demo


def _coerce_runner(scene_id: str, factory_result: Any) -> WorldRunner:
    if isinstance(factory_result, WorldRunner):
        return factory_result

    world = factory_result
    step_hook: Callable[[], None] | None = None

    info = getattr(factory_result, "info", None)
    if isinstance(info, dict):
        world = info.get("sx_world") or info.get("physics_world") or world

    setup_world = getattr(factory_result, "world", None)
    if info is not None and world is factory_result and setup_world is not None:
        world = setup_world

    pre_step = getattr(factory_result, "pre_step", None)
    if callable(pre_step):
        step_hook = pre_step
    else:
        setup_step = getattr(factory_result, "step", None)
        if callable(setup_step):
            step_hook = lambda: setup_step(1)

    if world is None:
        raise ValueError(f"scene {scene_id!r} did not provide a DART World")

    if step_hook is None:
        step_hook = lambda: world.step()

    if hasattr(world, "is_simulation_mode") and not bool(world.is_simulation_mode):
        world.enter_simulation_mode()

    return WorldRunner(scene_id=scene_id, world=world, step_once=step_hook)


def resolve_world_runner(
    *, scene: str | None = None, factory: str | None = None
) -> WorldRunner:
    """Resolve a built-in scene, demo-scene id, or importable factory."""

    if scene is not None and factory is not None:
        raise ValueError("pass either --scene or --factory, not both")

    if factory is not None:
        callable_factory = _load_factory(factory)
        return _coerce_runner(factory, callable_factory())

    scene_id = scene or "box_on_ground"
    callable_factory = _BUILTIN_SCENES.get(scene_id)
    if callable_factory is None:
        callable_factory = _demo_scene_factory(scene_id)
    return _coerce_runner(scene_id, callable_factory())


def _scene_json(world: Any) -> dict[str, Any]:
    sx = _import_dartpy()
    if not hasattr(sx, "dump_scene_json"):
        raise RuntimeError("dartpy.dump_scene_json is required for body discovery")
    return sx.dump_scene_json(world)


def _resolve_body(world: Any, body_entry: dict[str, Any]) -> Any | None:
    body_id = str(body_entry.get("id", ""))
    name = str(body_entry.get("name", ""))
    if body_id.startswith("body:") and "/" in body_id:
        multibody_name, link_name = body_id[len("body:") :].split("/", 1)
        multibody = world.get_multibody(multibody_name)
        if multibody is not None:
            return multibody.get_link(link_name)
        return None
    if hasattr(world, "get_rigid_body"):
        return world.get_rigid_body(name)
    return None


def _explicit_body(world: Any, label: str) -> Any | None:
    if "/" in label and hasattr(world, "get_multibody"):
        multibody_name, link_name = label.split("/", 1)
        multibody = world.get_multibody(multibody_name)
        if multibody is not None:
            return multibody.get_link(link_name)
    if hasattr(world, "get_rigid_body"):
        return world.get_rigid_body(label)
    return None


def discover_bodies(world: Any, only: Iterable[str] | None = None) -> list[TrackedBody]:
    requested = set(only or [])
    scene = _scene_json(world)
    tracked: list[TrackedBody] = []
    matched_requests: set[str] = set()
    for entry in scene.get("bodies", []):
        body_id = str(entry.get("id", ""))
        label = body_id[len("body:") :] if body_id.startswith("body:") else body_id
        name = str(entry.get("name", ""))
        candidates = {label, name, body_id}
        if requested and not requested.intersection(candidates):
            continue
        body = _resolve_body(world, entry)
        if body is not None:
            tracked.append(TrackedBody(_ensure_simple_label(label), body))
            matched_requests.update(requested.intersection(candidates))

    if not tracked and requested:
        for label in requested:
            body = _explicit_body(world, label)
            if body is not None:
                tracked.append(TrackedBody(_ensure_simple_label(label), body))
                matched_requests.add(label)

    missing = requested.difference(matched_requests)
    if missing:
        raise ValueError(
            f"requested bodies were not found: {', '.join(sorted(missing))}"
        )
    if not tracked:
        raise ValueError(
            "no trackable bodies found; create the world after importing dartpy "
            "or pass explicit --body names"
        )
    return tracked


def _header_lines(scene_id: str, world: Any) -> list[str]:
    scene = _scene_json(world)
    header = scene["world"]
    units = header.get("units", _UNITS)
    return [
        f"# {SCHEMA_VERSION} scene={scene_id}",
        f"# gravity: {_json_compact(header.get('gravity', _vector(world.gravity)))}",
        f"# time_step: {_fmt(header.get('time_step', world.time_step))}",
        f"# rest_tolerance: {_fmt(header.get('rest_tolerance', 0.0))}",
        f"# units: {_json_compact(units)}",
        "# columns: " + " ".join(TRAJECTORY_COLUMNS),
    ]


def _trajectory_row(
    frame: int, time: float, body: TrackedBody, contact_count: int
) -> str:
    position = _vector(body.body.translation)
    linear = _vector(body.body.linear_velocity)
    angular = _vector(body.body.angular_velocity)
    values = [
        str(frame),
        _fmt(time),
        body.label,
        *(_fmt(value) for value in position),
        *(_fmt(value) for value in linear),
        *(_fmt(value) for value in angular),
        str(int(contact_count)),
    ]
    return "\t".join(values)


def record_trajectory(
    runner: WorldRunner, steps: int, bodies: Iterable[str] = ()
) -> str:
    if steps < 0:
        raise ValueError("steps must be non-negative")
    tracked = discover_bodies(runner.world, bodies)
    lines = _header_lines(runner.scene_id, runner.world)
    for _ in range(steps):
        runner.step_once()
        contacts = runner.world.collide()
        frame = int(runner.world.frame)
        time = float(runner.world.time)
        for body in tracked:
            lines.append(_trajectory_row(frame, time, body, len(contacts)))
    return "\n".join(lines) + "\n"


def record_trajectory_and_contact_events(
    runner: WorldRunner, steps: int, bodies: Iterable[str] = ()
) -> tuple[str, str]:
    if steps < 0:
        raise ValueError("steps must be non-negative")
    tracked = discover_bodies(runner.world, bodies)
    trajectory_lines = _header_lines(runner.scene_id, runner.world)
    contact_lines: list[str] = []
    previous_count = 0
    for _ in range(steps):
        runner.step_once()
        contacts = _sorted_contacts(runner.world.collide())
        count = len(contacts)
        frame = int(runner.world.frame)
        time = float(runner.world.time)

        for body in tracked:
            trajectory_lines.append(_trajectory_row(frame, time, body, count))

        if count == 0 and previous_count == 0:
            previous_count = 0
            continue
        if previous_count == 0 and count > 0:
            event = "onset"
        elif previous_count > 0 and count == 0:
            event = "break"
        elif previous_count != count:
            event = "count_change"
        else:
            event = "steady"
        payload = {
            "schema_version": CONTACT_SCHEMA_VERSION,
            "scene": runner.scene_id,
            "frame": frame,
            "time": time,
            "event": event,
            "count": count,
            "contacts": contacts,
        }
        contact_lines.append(json.dumps(payload, sort_keys=True))
        previous_count = count

    trajectory_text = "\n".join(trajectory_lines) + "\n"
    contact_text = "\n".join(contact_lines)
    if contact_text:
        contact_text += "\n"
    return trajectory_text, contact_text


def record_trajectory_for_scene(
    scene: str, steps: int, bodies: Iterable[str] = ()
) -> str:
    return record_trajectory(resolve_world_runner(scene=scene), steps, bodies)


def _contact_body_name(body: Any) -> str:
    name = getattr(body, "name", None)
    if name is not None:
        return _ensure_simple_label(str(name))
    return _ensure_simple_label(repr(body))


def _contact_payload(contact: Any) -> dict[str, Any]:
    return {
        "body_a": _contact_body_name(contact.body_a),
        "body_b": _contact_body_name(contact.body_b),
        "position": _vector(contact.point),
        "normal": _vector(contact.normal),
        "depth": float(contact.depth),
        "shape_index_a": int(getattr(contact, "shape_index_a", -1)),
        "shape_index_b": int(getattr(contact, "shape_index_b", -1)),
    }


def _sorted_contacts(contacts: Iterable[Any]) -> list[dict[str, Any]]:
    payloads = [_contact_payload(contact) for contact in contacts]
    return sorted(
        payloads,
        key=lambda item: (
            item["body_a"],
            item["body_b"],
            item["shape_index_a"],
            item["shape_index_b"],
            item["depth"],
            item["position"],
        ),
    )


def record_contact_events(runner: WorldRunner, steps: int) -> str:
    if steps < 0:
        raise ValueError("steps must be non-negative")
    lines: list[str] = []
    previous_count = 0
    for _ in range(steps):
        runner.step_once()
        contacts = _sorted_contacts(runner.world.collide())
        count = len(contacts)
        if count == 0 and previous_count == 0:
            previous_count = 0
            continue
        if previous_count == 0 and count > 0:
            event = "onset"
        elif previous_count > 0 and count == 0:
            event = "break"
        elif previous_count != count:
            event = "count_change"
        else:
            event = "steady"
        payload = {
            "schema_version": CONTACT_SCHEMA_VERSION,
            "scene": runner.scene_id,
            "frame": int(runner.world.frame),
            "time": float(runner.world.time),
            "event": event,
            "count": count,
            "contacts": contacts,
        }
        lines.append(json.dumps(payload, sort_keys=True, separators=(",", ":")))
        previous_count = count
    return "\n".join(lines) + ("\n" if lines else "")


def record_contact_events_for_scene(scene: str, steps: int) -> str:
    return record_contact_events(resolve_world_runner(scene=scene), steps)


def _write_output(path: Path | None, text: str) -> None:
    if path is None:
        sys.stdout.write(text)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Record a DART 7 trajectory TSV or contact-event JSONL trace."
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--scene", default="box_on_ground")
    source.add_argument("--factory", help="importable world factory: module:callable")
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument(
        "--body",
        action="append",
        default=[],
        help="body label/name to track; repeat to select several bodies",
    )
    parser.add_argument(
        "--contacts",
        action="store_true",
        help="emit contact-event JSONL instead of trajectory TSV",
    )
    parser.add_argument(
        "--out", type=Path, help="write output to this path instead of stdout"
    )
    args = parser.parse_args(argv)

    try:
        runner = resolve_world_runner(scene=args.scene, factory=args.factory)
        if args.contacts:
            text = record_contact_events(runner, args.steps)
        else:
            text = record_trajectory(runner, args.steps, args.body)
        _write_output(args.out, text)
    except (ImportError, OSError, TypeError, ValueError, RuntimeError) as exc:
        print(f"trajectory_record.py: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
