"""Text and JSON scene dumps for DART 7 simulation worlds."""

from __future__ import annotations

import math
from functools import wraps
from typing import Any, Callable

import numpy as np

_WORLD_RIGID_BODY_NAMES: dict[int, list[str]] = {}
_WORLD_MULTIBODY_NAMES: dict[int, list[str]] = {}

_UNITS = {
    "length": "meter",
    "mass": "kilogram",
    "time": "second",
    "angle": "radian",
}


def _record_name(registry: dict[int, list[str]], owner: Any, name: str) -> None:
    if not name:
        return
    names = registry.setdefault(id(owner), [])
    if name not in names:
        names.append(name)


def _clear_world(world: Any) -> None:
    _WORLD_RIGID_BODY_NAMES.pop(id(world), None)
    _WORLD_MULTIBODY_NAMES.pop(id(world), None)


def _object_name(obj: Any, fallback: str = "") -> str:
    try:
        name = getattr(obj, "name")
    except Exception:  # noqa: BLE001
        return fallback
    return str(name)


def _wrap_method_once(owner: Any, name: str, wrapper: Callable[..., Any]) -> None:
    sentinel = f"_dartpy_scene_dump_wrapped_{name}"
    if getattr(owner, sentinel, False):
        return
    method = getattr(owner, name, None)
    if method is None:
        return
    try:
        setattr(owner, name, wrapper(method))
        setattr(owner, sentinel, True)
    except Exception:  # noqa: BLE001
        pass


def _install_world_scene_registry(simulation: Any) -> None:
    world_type = getattr(simulation, "World", None)
    if world_type is None:
        return

    def add_rigid_body_wrapper(add_rigid_body: Callable[..., Any]):
        @wraps(add_rigid_body)
        def wrapped(self: Any, *args: Any, **kwargs: Any) -> Any:
            body = add_rigid_body(self, *args, **kwargs)
            fallback = str(args[0]) if args else ""
            _record_name(_WORLD_RIGID_BODY_NAMES, self, _object_name(body, fallback))
            return body

        return wrapped

    def add_multibody_wrapper(add_multibody: Callable[..., Any]):
        @wraps(add_multibody)
        def wrapped(self: Any, *args: Any, **kwargs: Any) -> Any:
            multibody = add_multibody(self, *args, **kwargs)
            fallback = str(args[0]) if args else ""
            _record_name(
                _WORLD_MULTIBODY_NAMES, self, _object_name(multibody, fallback)
            )
            return multibody

        return wrapped

    def clear_wrapper(clear: Callable[..., Any]):
        @wraps(clear)
        def wrapped(self: Any, *args: Any, **kwargs: Any) -> Any:
            _clear_world(self)
            return clear(self, *args, **kwargs)

        return wrapped

    _wrap_method_once(world_type, "add_rigid_body", add_rigid_body_wrapper)
    _wrap_method_once(world_type, "add_multibody", add_multibody_wrapper)
    _wrap_method_once(world_type, "clear", clear_wrapper)


def _install_loader_registry(simulation: Any) -> None:
    for name in ("add_skeleton", "build_multibody_from_skeleton"):
        sentinel = f"_dartpy_scene_dump_wrapped_{name}"
        if getattr(simulation, sentinel, False):
            continue
        loader = getattr(simulation, name, None)
        if loader is None:
            continue

        @wraps(loader)
        def wrapped_loader(*args: Any, _loader=loader, **kwargs: Any) -> Any:
            multibody = _loader(*args, **kwargs)
            if args:
                world = args[0]
                _record_name(_WORLD_MULTIBODY_NAMES, world, _object_name(multibody, ""))
            return multibody

        try:
            setattr(simulation, name, wrapped_loader)
            setattr(simulation, sentinel, True)
        except Exception:  # noqa: BLE001
            pass


def install_scene_dump_helpers(root: Any, simulation: Any) -> None:
    """Attach scene dump helpers to the public simulation module."""

    _install_world_scene_registry(simulation)
    _install_loader_registry(simulation)
    simulation.dump_scene_json = dump_scene_json
    simulation.dump_scene_text = dump_scene_text
    # When this runs before layout promotion, _layout will publish these flat.
    # If a caller imports a nonstandard layout, make the flat helpers available.
    try:
        root.dump_scene_json = dump_scene_json
        root.dump_scene_text = dump_scene_text
    except Exception:  # noqa: BLE001
        pass


def _array(value: Any, *, shape: tuple[int, ...] | None = None) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if shape is not None:
        array = array.reshape(shape)
    return array


def _vector(value: Any, length: int) -> list[float]:
    return _array(value, shape=(length,)).astype(float).tolist()


def _matrix(value: Any, rows: int, cols: int) -> list[list[float]]:
    return _array(value, shape=(rows, cols)).astype(float).tolist()


def _safe_attr(obj: Any, name: str, default: Any = None) -> Any:
    try:
        return getattr(obj, name)
    except Exception:  # noqa: BLE001
        return default


def _enum_name(value: Any) -> str:
    name = _safe_attr(value, "name")
    if name is not None:
        return str(name).lower()
    text = str(value)
    if "." in text:
        return text.rsplit(".", 1)[-1].lower()
    return text.lower()


def _rotation_to_rpy(rotation: Any) -> list[float]:
    matrix = _array(rotation, shape=(3, 3))
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    singular = sy < 1e-12
    if not singular:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = 0.0
    return [float(roll), float(pitch), float(yaw)]


def _shape_size(shape: Any, shape_type: str) -> list[float] | None:
    if shape_type == "box":
        half_extents = _array(_safe_attr(shape, "half_extents"), shape=(3,))
        return (2.0 * half_extents).tolist()
    if shape_type == "sphere":
        radius = float(_safe_attr(shape, "radius", 0.0))
        diameter = 2.0 * radius
        return [diameter, diameter, diameter]
    if shape_type == "cylinder":
        radius = float(_safe_attr(shape, "radius", 0.0))
        half_height = float(_safe_attr(shape, "half_height", 0.0))
        return [2.0 * radius, 2.0 * radius, 2.0 * half_height]
    if shape_type == "capsule":
        radius = float(_safe_attr(shape, "radius", 0.0))
        half_height = float(_safe_attr(shape, "half_height", 0.0))
        # Full axial bounding extent includes the two hemispherical caps, so the
        # Z size is 2*half_height + 2*radius (matching the full-bbox convention
        # used for box/sphere/mesh, not the bare cylinder segment).
        return [2.0 * radius, 2.0 * radius, 2.0 * half_height + 2.0 * radius]
    vertices = _safe_attr(shape, "vertices")
    if vertices is not None:
        points = np.asarray(vertices, dtype=float)
        if points.size > 0:
            points = points.reshape((-1, 3))
            return (points.max(axis=0) - points.min(axis=0)).tolist()
    return None


def _shape_to_json(shape: Any, body_id: str, index: int) -> dict[str, Any]:
    shape_type = _enum_name(_safe_attr(shape, "type"))
    shape_id = f"shape:{body_id}:{index}"
    data: dict[str, Any] = {
        "id": shape_id,
        "type": shape_type,
        "local_transform": _matrix(_safe_attr(shape, "local_transform"), 4, 4),
    }
    size = _shape_size(shape, shape_type)
    if size is not None:
        data["size"] = size
    if shape_type == "box":
        data["half_extents"] = _vector(_safe_attr(shape, "half_extents"), 3)
    elif shape_type == "sphere":
        data["radius"] = float(_safe_attr(shape, "radius", 0.0))
    elif shape_type in {"capsule", "cylinder"}:
        data["radius"] = float(_safe_attr(shape, "radius", 0.0))
        data["half_height"] = float(_safe_attr(shape, "half_height", 0.0))
        data["height"] = float(_safe_attr(shape, "height", 0.0))
    elif shape_type == "plane":
        data["normal"] = _vector(_safe_attr(shape, "normal"), 3)
        data["offset"] = float(_safe_attr(shape, "offset", 0.0))
    elif shape_type == "mesh":
        vertices = _safe_attr(shape, "vertices")
        triangles = _safe_attr(shape, "triangles")
        data["vertex_count"] = int(len(vertices) if vertices is not None else 0)
        data["triangle_count"] = int(len(triangles) if triangles is not None else 0)
    return data


def _body_to_json(
    body: Any,
    *,
    body_id: str,
    kind: str,
    joint: dict[str, Any] | None = None,
) -> dict[str, Any]:
    rotation = _safe_attr(body, "rotation", np.eye(3))
    transform = _safe_attr(body, "transform", np.eye(4))
    inertia = _array(_safe_attr(body, "inertia", np.zeros((3, 3))), shape=(3, 3))
    shapes = [
        _shape_to_json(shape, body_id, index)
        for index, shape in enumerate(list(_safe_attr(body, "collision_shapes", [])))
    ]
    position = _vector(_safe_attr(body, "translation", np.zeros(3)), 3)
    orientation_rpy = _rotation_to_rpy(rotation)
    has_collision_shape = bool(_safe_attr(body, "has_collision_shape", bool(shapes)))

    data: dict[str, Any] = {
        "id": body_id,
        "name": _object_name(body),
        "kind": kind,
        "is_static": bool(_safe_attr(body, "is_static", False)),
        "is_kinematic": bool(_safe_attr(body, "is_kinematic", False)),
        "mass": float(_safe_attr(body, "mass", 0.0)),
        "inertia": inertia.tolist(),
        "inertia_diag": np.diag(inertia).astype(float).tolist(),
        "position": position,
        "orientation_rpy": orientation_rpy,
        "pose": {
            "position": position,
            "orientation_rpy": orientation_rpy,
            "transform": _matrix(transform, 4, 4),
        },
        "linear_velocity": _vector(_safe_attr(body, "linear_velocity", np.zeros(3)), 3),
        "angular_velocity": _vector(
            _safe_attr(body, "angular_velocity", np.zeros(3)), 3
        ),
        "has_collision_shape": has_collision_shape,
        "collision_shape": shapes[0] if shapes else None,
        "shapes": shapes,
        "joint": joint,
    }
    return data


def _joint_endpoint(joint: Any, rigid_name: str, link_name: str) -> str | None:
    body = _safe_attr(joint, rigid_name)
    if body is not None:
        name = _object_name(body)
        if name:
            return f"body:{name}"
    link = _safe_attr(joint, link_name)
    if link is not None:
        name = _object_name(link)
        if name:
            return f"link:{name}"
    return None


def _joint_to_json(joint: Any, *, owner: str = "world") -> dict[str, Any]:
    name = _object_name(joint)
    joint_id = f"joint:{owner}:{name}" if owner != "world" else f"joint:{name}"
    lower = _safe_attr(joint, "position_lower_limits")
    upper = _safe_attr(joint, "position_upper_limits")
    data: dict[str, Any] = {
        "id": joint_id,
        "name": name,
        "type": _enum_name(_safe_attr(joint, "type")),
        "position": _array(_safe_attr(joint, "position", [])).reshape(-1).tolist(),
        "limits": {
            "lower": _array(lower).reshape(-1).tolist() if lower is not None else [],
            "upper": _array(upper).reshape(-1).tolist() if upper is not None else [],
        },
        "parent": _joint_endpoint(joint, "parent_rigid_body", "parent_link"),
        "child": _joint_endpoint(joint, "child_rigid_body", "child_link"),
    }
    return data


def _tracked_rigid_bodies(world: Any) -> list[Any]:
    get_rigid_body = _safe_attr(world, "get_rigid_body")
    if get_rigid_body is None:
        return []
    bodies = []
    for name in _WORLD_RIGID_BODY_NAMES.get(id(world), []):
        body = get_rigid_body(name)
        if body is not None:
            bodies.append(body)
    return bodies


def _tracked_multibodies(world: Any) -> list[Any]:
    get_multibody = _safe_attr(world, "get_multibody")
    if get_multibody is None:
        return []
    multibodies = []
    for name in _WORLD_MULTIBODY_NAMES.get(id(world), []):
        multibody = get_multibody(name)
        if multibody is not None:
            multibodies.append(multibody)
    return multibodies


def _solver_rest_tolerance(world: Any) -> float:
    options = _safe_attr(world, "multibody_options")
    return float(_safe_attr(options, "variational_tolerance", 0.0))


def _world_header(world: Any, body_count: int, joint_count: int) -> dict[str, Any]:
    return {
        "id": "world",
        "gravity": _vector(_safe_attr(world, "gravity", np.zeros(3)), 3),
        "time_step": float(_safe_attr(world, "time_step", 0.0)),
        "time": float(_safe_attr(world, "time", 0.0)),
        "frame": int(_safe_attr(world, "frame", 0)),
        "units": dict(_UNITS),
        "rest_tolerance": _solver_rest_tolerance(world),
        "body_count": body_count,
        "joint_count": joint_count,
        "rigid_body_count": int(_safe_attr(world, "num_rigid_bodies", 0)),
        "multibody_count": int(_safe_attr(world, "num_multibodies", 0)),
    }


def _format_vector(values: Any) -> str:
    return "[" + ", ".join(f"{float(value):.6g}" for value in values) + "]"


def _flat_body_summary(body: dict[str, Any]) -> str:
    return (
        f"{body['kind']} body {body['name']} mass={body['mass']:.6g} "
        f"position={_format_vector(body['position'])} "
        f"shapes={len(body['shapes'])} "
        f"collision_shape={str(body['has_collision_shape']).lower()}"
    )


def _flat_shape_summary(shape: dict[str, Any], body_name: str) -> str:
    summary = f"{shape['type']} shape on {body_name}"
    if "size" in shape:
        summary += f" size={_format_vector(shape['size'])}"
    return summary


def _flat_joint_summary(joint: dict[str, Any]) -> str:
    return (
        f"{joint['type']} joint {joint['name']} "
        f"position={_format_vector(joint['position'])} "
        f"limits={_format_vector(joint['limits']['lower'])}:"
        f"{_format_vector(joint['limits']['upper'])}"
    )


def _build_flat_index(scene: dict[str, Any]) -> dict[str, str]:
    header = scene["world"]
    index = {
        "world": (
            f"world bodies={header['body_count']} joints={header['joint_count']} "
            f"time_step={header['time_step']:.6g} "
            f"gravity={_format_vector(header['gravity'])}"
        )
    }
    for body in scene["bodies"]:
        index[body["id"]] = _flat_body_summary(body)
        for shape in body["shapes"]:
            index[shape["id"]] = _flat_shape_summary(shape, body["name"])
    for joint in scene["joints"]:
        index[joint["id"]] = _flat_joint_summary(joint)
    return index


def dump_scene_json(world: Any) -> dict[str, Any]:
    """Return a JSON-serializable DART scene description for a World."""

    bodies: list[dict[str, Any]] = []
    joints: list[dict[str, Any]] = []

    for body in _tracked_rigid_bodies(world):
        body_id = f"body:{_object_name(body)}"
        bodies.append(_body_to_json(body, body_id=body_id, kind="rigid"))

    for joint in list(_safe_attr(world, "joints", [])):
        joints.append(_joint_to_json(joint))

    for multibody in _tracked_multibodies(world):
        multibody_name = _object_name(multibody, "multibody")
        joint_by_child = {
            _object_name(_safe_attr(joint, "child_link"), ""): _joint_to_json(
                joint, owner=multibody_name
            )
            for joint in list(_safe_attr(multibody, "joints", []))
        }
        for link in list(_safe_attr(multibody, "links", [])):
            link_name = _object_name(link)
            joint = joint_by_child.get(link_name)
            body_id = f"body:{multibody_name}/{link_name}"
            bodies.append(
                _body_to_json(link, body_id=body_id, kind="link", joint=joint)
            )
        joints.extend(joint_by_child.values())

    scene = {
        "schema": "dart.scene/v0",
        "world": _world_header(world, len(bodies), len(joints)),
        "bodies": bodies,
        "joints": joints,
    }
    scene["flat_index"] = _build_flat_index(scene)
    return scene


def dump_scene_text(world: Any) -> str:
    """Return a compact human- and agent-readable DART scene description."""

    scene = dump_scene_json(world)
    header = scene["world"]
    lines = [
        (
            "# dart.scene/v0 "
            f"time={header['time']:.6g} "
            f"time_step={header['time_step']:.6g} "
            f"frame={header['frame']} "
            f"gravity={_format_vector(header['gravity'])} "
            f"rest_tolerance={header['rest_tolerance']:.6g} "
            "units=m,kg,s,rad"
        ),
        f"world bodies={header['body_count']} joints={header['joint_count']}",
    ]
    for body in scene["bodies"]:
        lines.append(f"{body['id']} {_flat_body_summary(body)}")
        for shape in body["shapes"]:
            lines.append(f"  {shape['id']} {_flat_shape_summary(shape, body['name'])}")
    for joint in scene["joints"]:
        lines.append(f"{joint['id']} {_flat_joint_summary(joint)}")
    lines.append("flat_index:")
    for item_id, summary in scene["flat_index"].items():
        lines.append(f"  {item_id}: {summary}")
    return "\n".join(lines)
