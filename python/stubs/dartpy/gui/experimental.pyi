from __future__ import annotations

import typing

import dartpy.collision
import dartpy.dynamics
import dartpy.simulation
import numpy

__all__: list[str] = [
    'ActiveRenderableState',
    'DebugDrawOptions',
    'DebugLineDescriptor',
    'GeometryDescriptor',
    'MaterialDescriptor',
    'MeshAlphaMode',
    'MeshMaterialDescriptor',
    'MeshPartDescriptor',
    'OrbitCamera',
    'OrbitCameraBasis',
    'OrbitCameraUpdate',
    'PerspectiveProjection',
    'PickHit',
    'PickRay',
    'ProjectionOptions',
    'RenderableDescriptor',
    'RenderableSetUpdatePlan',
    'RunOptions',
    'ShapeKind',
    'ViewerLifecycleState',
    'camera_eye',
    'compute_plane_drag_translation',
    'describe_shape',
    'extract_contact_debug_lines',
    'extract_debug_lines',
    'extract_renderables',
    'intersect_plane',
    'intersect_renderable',
    'make_collision_shape_debug_lines',
    'make_frame_debug_lines',
    'make_grid_debug_lines',
    'make_inertia_debug_lines',
    'make_orbit_camera_basis',
    'make_perspective_pick_ray',
    'make_perspective_projection',
    'make_selection_debug_lines',
    'make_support_polygon_debug_lines',
    'mark_frame_rendered',
    'mark_frame_skipped',
    'mark_screenshot_requested',
    'mark_simulation_advanced',
    'normalize_run_options',
    'pick_nearest_renderable',
    'plan_renderable_set_update',
    'request_single_step',
    'should_advance_simulation',
    'should_request_screenshot',
    'should_stop_after_frame',
    'toggle_paused',
    'translate_frame_renderable',
    'translate_free_joint_renderable',
    'translate_simple_frame_renderable',
    'update_orbit_camera',
    'write_rgba_ppm',
]


Array = numpy.ndarray


class ShapeKind:
    Box: typing.ClassVar[ShapeKind]
    Sphere: typing.ClassVar[ShapeKind]
    Ellipsoid: typing.ClassVar[ShapeKind]
    Cylinder: typing.ClassVar[ShapeKind]
    Capsule: typing.ClassVar[ShapeKind]
    Cone: typing.ClassVar[ShapeKind]
    Pyramid: typing.ClassVar[ShapeKind]
    MultiSphere: typing.ClassVar[ShapeKind]
    LineSegments: typing.ClassVar[ShapeKind]
    ConvexMesh: typing.ClassVar[ShapeKind]
    PointCloud: typing.ClassVar[ShapeKind]
    Heightmap: typing.ClassVar[ShapeKind]
    SoftMesh: typing.ClassVar[ShapeKind]
    VoxelGrid: typing.ClassVar[ShapeKind]
    Mesh: typing.ClassVar[ShapeKind]
    Plane: typing.ClassVar[ShapeKind]
    Unsupported: typing.ClassVar[ShapeKind]
    __members__: typing.ClassVar[dict[str, ShapeKind]]

    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...


class MeshAlphaMode:
    Blend: typing.ClassVar[MeshAlphaMode]
    Auto: typing.ClassVar[MeshAlphaMode]
    ShapeAlpha: typing.ClassVar[MeshAlphaMode]
    __members__: typing.ClassVar[dict[str, MeshAlphaMode]]

    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...


class MeshMaterialDescriptor:
    ambient: Array
    diffuse: Array
    specular: Array
    emissive: Array
    shininess: float
    metallic_factor: float
    roughness_factor: float
    base_color_texture_path: str
    metallic_texture_path: str
    roughness_texture_path: str
    metallic_roughness_texture_path: str
    normal_texture_path: str
    occlusion_texture_path: str
    emissive_texture_path: str
    texture_image_paths: list[str]

    def __init__(self) -> None: ...


class MeshPartDescriptor:
    vertex_offset: int
    vertex_count: int
    triangle_offset: int
    triangle_count: int
    material_index: int

    def __init__(self) -> None: ...


class GeometryDescriptor:
    kind: ShapeKind
    size: Array
    scale: Array
    normal: Array
    local_bounds_min: Array
    local_bounds_max: Array
    sphere_centers: list[Array]
    sphere_radii: list[float]
    line_vertices: list[Array]
    line_connections: list[Array]
    triangle_vertices: list[Array]
    triangle_indices: list[Array]
    triangle_normals: list[Array]
    point_cloud_points: list[Array]
    point_cloud_colors: list[Array]
    voxel_centers: list[Array]
    radius: float
    height: float
    offset: float
    line_thickness: float
    point_size: float
    voxel_size: float
    has_local_bounds: bool
    mesh_uses_material_colors: bool
    mesh_alpha_mode: MeshAlphaMode
    mesh_texture_coord_components: int
    mesh_texture_coordinates: list[Array]
    mesh_uri: str
    shape_type: str
    unsupported_reason: str
    mesh_materials: list[MeshMaterialDescriptor]
    mesh_parts: list[MeshPartDescriptor]

    def __init__(self) -> None: ...


class MaterialDescriptor:
    rgba: Array
    visible: bool
    casts_shadows: bool
    receives_shadows: bool

    def __init__(self) -> None: ...


class RenderableDescriptor:
    id: int
    skeleton_name: str
    body_name: str
    shape_frame_name: str
    shape_node_name: str
    geometry: GeometryDescriptor
    material: MaterialDescriptor
    world_transform: Array
    shape_frame_version: int
    shape_node_version: int
    shape_version: int
    render_resource_version: int

    def __init__(self) -> None: ...


class RenderableSetUpdatePlan:
    descriptor_indices_to_add: list[int]
    active_renderable_indices_to_remove: list[int]

    def __init__(self) -> None: ...


class ActiveRenderableState:
    id: int
    shape_version: int
    render_resource_version: int

    def __init__(self) -> None: ...


class PickRay:
    origin: Array
    direction: Array

    def __init__(self) -> None: ...


class RunOptions:
    width: int
    height: int
    max_frames: int
    gui_scale: float
    headless: bool
    screenshot_path: str

    def __init__(self) -> None: ...


class ViewerLifecycleState:
    rendered_frames: int
    skipped_frames: int
    paused: bool
    step_once: bool
    screenshot_requested: bool

    def __init__(self) -> None: ...


class OrbitCamera:
    target: Array
    yaw: float
    pitch: float
    distance: float

    def __init__(self) -> None: ...


class OrbitCameraBasis:
    eye: Array
    forward: Array
    right: Array
    up: Array

    def __init__(self) -> None: ...


class OrbitCameraUpdate:
    delta_x: float
    delta_y: float
    scroll_delta: float
    orbit: bool
    pan: bool
    orbit_scale: float
    pan_scale: float
    scroll_scale: float
    min_distance: float
    max_distance: float
    min_pitch: float
    max_pitch: float

    def __init__(self) -> None: ...


class DirectionalNudgeInput:
    left: bool
    right: bool
    forward: bool
    backward: bool
    up: bool
    down: bool
    fast: bool
    step_size: float
    fast_multiplier: float

    def __init__(self) -> None: ...


class ProjectionOptions:
    vertical_fov_degrees: float
    near_plane: float | None
    far_plane: float | None
    near_scale: float
    min_near_plane: float
    max_near_plane: float
    min_far_plane: float
    far_padding: float

    def __init__(self) -> None: ...


class PerspectiveProjection:
    vertical_fov_degrees: float
    aspect_ratio: float
    near_plane: float
    far_plane: float

    def __init__(self) -> None: ...


class PickHit:
    id: int
    renderable_index: int
    distance: float
    point: Array
    normal: Array

    def __init__(self) -> None: ...


class DebugLineDescriptor:
    from_point: Array
    to_point: Array
    rgba: Array
    label: str

    def __init__(self) -> None: ...


class DebugDrawOptions:
    draw_grid: bool
    draw_world_frame: bool
    draw_body_frames: bool
    draw_centers_of_mass: bool
    draw_inertia_boxes: bool
    draw_collision_shape_bounds: bool
    draw_support_polygons: bool
    draw_support_centroids: bool
    draw_contacts: bool
    draw_contact_normals: bool
    draw_contact_forces: bool
    grid_half_extent: float
    grid_spacing: float
    grid_z: float
    world_frame_axis_length: float
    body_frame_axis_length: float
    center_of_mass_marker_radius: float
    inertia_box_scale: float
    collision_bounds_padding: float
    support_polygon_elevation: float
    support_centroid_marker_radius: float
    contact_marker_half_extent: float
    contact_normal_length: float
    contact_force_scale: float
    contact_force_min_length: float
    contact_force_max_length: float

    def __init__(self) -> None: ...


def describe_shape(shape: dartpy.dynamics.Shape) -> GeometryDescriptor | None: ...
def extract_renderables(world: dartpy.simulation.World) -> list[RenderableDescriptor]: ...
@typing.overload
def plan_renderable_set_update(
    descriptors: list[RenderableDescriptor], active_renderable_ids: list[int]
) -> RenderableSetUpdatePlan: ...
@typing.overload
def plan_renderable_set_update(
    descriptors: list[RenderableDescriptor],
    active_renderable_states: list[ActiveRenderableState],
) -> RenderableSetUpdatePlan: ...
def intersect_renderable(
    renderable: RenderableDescriptor, ray: PickRay
) -> float | None: ...
def pick_nearest_renderable(
    renderables: list[RenderableDescriptor],
    ray: PickRay,
    max_distance: float = ...,
) -> PickHit | None: ...
def intersect_plane(
    ray: PickRay, plane_point: Array, plane_normal: Array
) -> Array | None: ...
def compute_plane_drag_translation(
    previous_ray: PickRay,
    current_ray: PickRay,
    plane_point: Array,
    plane_normal: Array,
) -> Array | None: ...
def translate_free_joint_renderable(
    renderable: RenderableDescriptor, world_translation: Array
) -> bool: ...
def translate_simple_frame_renderable(
    renderable: RenderableDescriptor, world_translation: Array
) -> bool: ...
def translate_frame_renderable(
    renderable: RenderableDescriptor, world_translation: Array
) -> bool: ...
def normalize_run_options(options: RunOptions) -> None: ...
@typing.overload
def should_request_screenshot(
    options: RunOptions, rendered_frames: int, screenshot_requested: bool
) -> bool: ...
@typing.overload
def should_request_screenshot(
    options: RunOptions, state: ViewerLifecycleState
) -> bool: ...
@typing.overload
def should_stop_after_frame(options: RunOptions, rendered_frames: int) -> bool: ...
@typing.overload
def should_stop_after_frame(
    options: RunOptions, state: ViewerLifecycleState
) -> bool: ...
def toggle_paused(state: ViewerLifecycleState) -> None: ...
def request_single_step(state: ViewerLifecycleState, pause: bool = ...) -> None: ...
def should_advance_simulation(state: ViewerLifecycleState) -> bool: ...
def mark_simulation_advanced(state: ViewerLifecycleState) -> None: ...
def mark_screenshot_requested(state: ViewerLifecycleState) -> None: ...
def mark_frame_rendered(state: ViewerLifecycleState) -> None: ...
def mark_frame_skipped(state: ViewerLifecycleState) -> None: ...
def write_rgba_ppm(
    path: str,
    width: int,
    height: int,
    rgba_pixels: list[int],
    origin_bottom_left: bool = ...,
) -> None: ...
def make_orbit_camera_basis(camera: OrbitCamera) -> OrbitCameraBasis: ...
def camera_eye(camera: OrbitCamera) -> Array: ...
def update_orbit_camera(camera: OrbitCamera, update: OrbitCameraUpdate) -> None: ...
def compute_camera_relative_nudge(
    camera: OrbitCamera, input: DirectionalNudgeInput
) -> Array: ...
def make_perspective_pick_ray(
    camera: OrbitCamera,
    cursor_x: float,
    cursor_y: float,
    width: int,
    height: int,
    vertical_fov_radians: float = ...,
) -> PickRay: ...
def make_perspective_projection(
    camera: OrbitCamera,
    width: int,
    height: int,
    options: ProjectionOptions = ...,
) -> PerspectiveProjection: ...
def make_grid_debug_lines(
    options: DebugDrawOptions = ...,
) -> list[DebugLineDescriptor]: ...
def make_frame_debug_lines(
    transform: Array, axis_length: float, label_prefix: str = ...
) -> list[DebugLineDescriptor]: ...
def make_selection_debug_lines(
    renderable: RenderableDescriptor,
    rgba: Array = ...,
    label_prefix: str = ...,
) -> list[DebugLineDescriptor]: ...
def make_inertia_debug_lines(
    body_node: dartpy.dynamics.BodyNode,
    options: DebugDrawOptions = ...,
    label_prefix: str = ...,
) -> list[DebugLineDescriptor]: ...
def make_collision_shape_debug_lines(
    shape_node: dartpy.dynamics.ShapeNode,
    options: DebugDrawOptions = ...,
    label_prefix: str = ...,
) -> list[DebugLineDescriptor]: ...
def make_support_polygon_debug_lines(
    skeleton: dartpy.dynamics.Skeleton,
    options: DebugDrawOptions = ...,
    label_prefix: str = ...,
) -> list[DebugLineDescriptor]: ...
def extract_contact_debug_lines(
    result: dartpy.collision.CollisionResult,
    options: DebugDrawOptions = ...,
) -> list[DebugLineDescriptor]: ...
def extract_debug_lines(
    world: dartpy.simulation.World,
    options: DebugDrawOptions = ...,
) -> list[DebugLineDescriptor]: ...
