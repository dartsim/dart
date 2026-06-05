"""Shared bridge + grid builder for the IPC deformable demos.

The Filament viewer renders a `dart.simulation.World`, while these demos build
deformable physics through the current World API. `_world_bridge.WorldRenderBridge`
mirrors rigid bodies onto SimpleFrames by syncing a single transform per body.
Deformable bodies instead move every node independently, so this bridge mirrors
each deformable as:

- one `dart.SphereShape` SimpleFrame per node (orange for fixed/scripted nodes,
  blue for free nodes), its transform synced from ``body.node_position(i)``;
- one `dart.LineSegmentShape` SimpleFrame whose vertices track the nodes and
  whose connections are the body's spring edges (a wireframe of the mesh).

This is the Python equivalent of the C++ ``appendDeformableVisual`` /
``syncRenderFrames`` layers in
``examples/demos/scenes/deformable_body.cpp`` (per-node spheres + edge
wireframe). The dynamic *surface mesh* layer (``makeDeformableSurfaceRenderable``)
is not reachable through ``dartpy.gui.run_demos`` today, so it is intentionally
omitted; the spheres + wireframe convey sag, drape, and scripted motion.

Scenes build a deformable grid with :func:`build_grid_options`, add it to an
World, then register it with an :class:`IpcDeformableBridge` and return
``SceneSetup(world=bridge.render_world, pre_step=bridge.pre_step, ...)``.
"""

from __future__ import annotations

import math
import sys
from collections import deque
from typing import Any, Callable, Iterable, Sequence

import dartpy as dart
import dartpy as sx
import numpy as np

# Node-sphere radii and colors mirror the C++ deformable demo.
_FIXED_NODE_RADIUS = 0.045
_FREE_NODE_RADIUS = 0.032
_FIXED_NODE_COLOR = (0.95, 0.50, 0.16)
_FREE_NODE_COLOR = (0.12, 0.57, 0.91)
_EDGE_COLOR = (0.08, 0.13, 0.17)
_EDGE_THICKNESS = 2.8


def _translation(point: Any) -> np.ndarray:
    """A 4x4 homogeneous transform with the given translation."""

    matrix = np.eye(4)
    matrix[:3, 3] = np.asarray(point, dtype=float)
    return matrix


PositionFn = Callable[[int, int], Sequence[float]]
VelocityFn = Callable[[int, int], Sequence[float]]


def grid_index(columns: int, col: int, row: int) -> int:
    """Row-major node index for a ``columns``-wide grid."""

    return row * columns + col


def _grid_edges(
    columns: int, rows: int, positions: list[np.ndarray]
) -> list["sx.DeformableEdge"]:
    """Structural (H/V) plus shear (two diagonals per cell) spring edges.

    Rest lengths are the initial inter-node distances (what the C++ builder's
    ``-1.0`` auto-rest-length sentinel resolves to), so the cloth starts at
    rest in its initial layout.
    """

    edges: list[sx.DeformableEdge] = []

    def add(a: int, b: int) -> None:
        rest = float(np.linalg.norm(positions[a] - positions[b]))
        edges.append(sx.DeformableEdge(a, b, rest))

    for row in range(rows):
        for col in range(columns):
            here = grid_index(columns, col, row)
            if col + 1 < columns:
                add(here, grid_index(columns, col + 1, row))
            if row + 1 < rows:
                add(here, grid_index(columns, col, row + 1))
            if col + 1 < columns and row + 1 < rows:
                add(here, grid_index(columns, col + 1, row + 1))
                add(
                    grid_index(columns, col + 1, row),
                    grid_index(columns, col, row + 1),
                )
    return edges


def _grid_surface_triangles(
    columns: int, rows: int
) -> list["sx.DeformableSurfaceTriangle"]:
    """Two consistently-wound triangles per grid cell."""

    triangles: list[sx.DeformableSurfaceTriangle] = []
    for row in range(rows - 1):
        for col in range(columns - 1):
            a = grid_index(columns, col, row)
            b = grid_index(columns, col + 1, row)
            c = grid_index(columns, col, row + 1)
            d = grid_index(columns, col + 1, row + 1)
            triangles.append(sx.DeformableSurfaceTriangle(a, b, c))
            triangles.append(sx.DeformableSurfaceTriangle(b, d, c))
    return triangles


def build_grid_options(
    columns: int,
    rows: int,
    *,
    position_fn: PositionFn,
    mass: float,
    edge_stiffness: float,
    damping: float,
    velocity_fn: VelocityFn | None = None,
    fixed_nodes: Iterable[int] = (),
) -> "sx.DeformableBodyOptions":
    """A point-mass/spring grid as ``DeformableBodyOptions``.

    ``position_fn(col, row)`` and the optional ``velocity_fn(col, row)`` lay out
    the nodes; the topology (spring edges + surface triangles) is the shared
    grid pattern. This is the single seam every IPC deformable demo builds on.
    """

    if columns < 2 or rows < 2:
        raise ValueError("a deformable grid needs at least 2x2 nodes")

    options = sx.DeformableBodyOptions()
    options.edge_stiffness = edge_stiffness
    options.damping = damping

    positions: list[np.ndarray] = []
    velocities: list[np.ndarray] = []
    masses: list[float] = []
    for row in range(rows):
        for col in range(columns):
            positions.append(np.asarray(position_fn(col, row), dtype=float))
            velocity = velocity_fn(col, row) if velocity_fn else (0.0, 0.0, 0.0)
            velocities.append(np.asarray(velocity, dtype=float))
            masses.append(mass)

    options.positions = positions
    options.velocities = velocities
    options.masses = masses
    options.edges = _grid_edges(columns, rows, positions)
    options.surface_triangles = _grid_surface_triangles(columns, rows)
    fixed = list(fixed_nodes)
    if fixed:
        options.fixed_nodes = fixed
    return options


# The 6-tetrahedron (Kuhn) decomposition of a hexahedral cell, indexed by the
# cell's eight corners along the main diagonal corner0 -> corner7. |det| rest
# volumes make per-tet orientation irrelevant to the FEM energy.
_CUBE_TETRAHEDRA = (
    (0, 1, 3, 7),
    (0, 3, 2, 7),
    (0, 2, 6, 7),
    (0, 6, 4, 7),
    (0, 4, 5, 7),
    (0, 5, 1, 7),
)


def _fem_bar_mesh(
    cells_x: int,
    cells_y: int,
    cells_z: int,
    cell_size: float,
    origin: Sequence[float],
) -> tuple[
    list[np.ndarray],
    list["sx.DeformableTetrahedron"],
    list[tuple[int, int]],
    Callable[[int, int, int], int],
    tuple[int, int, int],
]:
    """Shared hexahedral->tetrahedral mesh for the FEM bar scenes.

    Returns node positions, tetrahedra, the unique tet edge list (render
    wireframe), the ``node_index(i, j, k)`` accessor, and the ``(nx, ny, nz)``
    node-grid dimensions.
    """

    nx, ny, nz = cells_x + 1, cells_y + 1, cells_z + 1

    def node_index(i: int, j: int, k: int) -> int:
        return i + nx * (j + ny * k)

    positions: list[np.ndarray] = []
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                positions.append(
                    np.asarray(
                        (
                            origin[0] + i * cell_size,
                            origin[1] + j * cell_size,
                            origin[2] + k * cell_size,
                        ),
                        dtype=float,
                    )
                )

    tetrahedra: list[sx.DeformableTetrahedron] = []
    edges: set[tuple[int, int]] = set()
    for k in range(cells_z):
        for j in range(cells_y):
            for i in range(cells_x):
                # Cube corners in bit order (x, y, z): corner b has
                # x = b&1, y = (b>>1)&1, z = (b>>2)&1.
                corner = [
                    node_index(i + (b & 1), j + ((b >> 1) & 1), k + ((b >> 2) & 1))
                    for b in range(8)
                ]
                for a, b, c, d in _CUBE_TETRAHEDRA:
                    n0, n1, n2, n3 = corner[a], corner[b], corner[c], corner[d]
                    tetrahedra.append(sx.DeformableTetrahedron(n0, n1, n2, n3))
                    for u, v in (
                        (n0, n1),
                        (n0, n2),
                        (n0, n3),
                        (n1, n2),
                        (n1, n3),
                        (n2, n3),
                    ):
                        edges.add((u, v) if u < v else (v, u))

    return positions, tetrahedra, sorted(edges), node_index, (nx, ny, nz)


def _fem_bar_options(
    positions: list[np.ndarray],
    tetrahedra: list["sx.DeformableTetrahedron"],
    youngs_modulus: float,
    poisson_ratio: float,
    density: float,
    use_fixed_corotational: bool = False,
) -> "sx.DeformableBodyOptions":
    options = sx.DeformableBodyOptions()
    options.positions = positions
    options.tetrahedra = tetrahedra
    options.material.youngs_modulus = youngs_modulus
    options.material.poisson_ratio = poisson_ratio
    options.material.density = density
    options.material.use_finite_element_elasticity = True
    options.material.use_fixed_corotational_elasticity = use_fixed_corotational
    return options


def build_fem_bar(
    *,
    cells_x: int,
    cells_y: int,
    cells_z: int,
    cell_size: float,
    origin: Sequence[float],
    youngs_modulus: float,
    poisson_ratio: float = 0.3,
    density: float = 1.0e3,
) -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    """A tetrahedralized FEM beam pinned at its ``x == 0`` face.

    Returns the ``DeformableBodyOptions`` (opting in to stable neo-Hookean FEM
    elasticity, with tetrahedra and no spring edges) plus the unique tet edge
    list for the render wireframe.
    """

    positions, tetrahedra, edges, node_index, (_, ny, nz) = _fem_bar_mesh(
        cells_x, cells_y, cells_z, cell_size, origin
    )
    options = _fem_bar_options(
        positions, tetrahedra, youngs_modulus, poisson_ratio, density
    )
    options.fixed_nodes = [node_index(0, j, k) for k in range(nz) for j in range(ny)]
    return options, edges


def build_fem_twist_bar(
    *,
    cells_x: int,
    cells_y: int,
    cells_z: int,
    cell_size: float,
    origin: Sequence[float],
    youngs_modulus: float,
    twist_rate: float,
    twist_end_time: float,
    poisson_ratio: float = 0.3,
    density: float = 1.0e3,
    use_fixed_corotational: bool = False,
) -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    """A FEM beam twisted at both ends by opposing scripted rotations.

    The two end faces are driven by ``DeformableDirichletBoundaryCondition``s
    that counter-rotate about the bar's long (x) axis until ``twist_end_time``,
    then release; the elastic FEM core resists the shear and untwists. The
    scripted drive uses a linear ``omega x r`` extrapolation, so ``twist_rate``
    and ``twist_end_time`` are kept small to stay near a true rotation. Toward
    the IPC paper's Fig. 4 (rod twist) / Fig. 14 (mat twist) themes. Set
    ``use_fixed_corotational`` to drive the bar with the fixed-corotational
    material instead of the default stable neo-Hookean kernel.
    """

    positions, tetrahedra, edges, node_index, (nx, ny, nz) = _fem_bar_mesh(
        cells_x, cells_y, cells_z, cell_size, origin
    )
    options = _fem_bar_options(
        positions,
        tetrahedra,
        youngs_modulus,
        poisson_ratio,
        density,
        use_fixed_corotational=use_fixed_corotational,
    )

    axis_y = origin[1] + 0.5 * cells_y * cell_size
    axis_z = origin[2] + 0.5 * cells_z * cell_size

    def end_face(i: int) -> list[int]:
        return [node_index(i, j, k) for k in range(nz) for j in range(ny)]

    conditions = []
    for face_i, sign in ((0, 1.0), (nx - 1, -1.0)):
        condition = sx.DeformableDirichletBoundaryCondition()
        condition.nodes = end_face(face_i)
        condition.angular_velocity = np.array([sign * twist_rate, 0.0, 0.0])
        condition.center = np.array([origin[0] + face_i * cell_size, axis_y, axis_z])
        condition.start_time = 0.0
        condition.end_time = twist_end_time
        conditions.append(condition)
    options.dirichlet_boundary_conditions = conditions
    return options, edges


def build_fem_compression_bar(
    *,
    cells_x: int,
    cells_y: int,
    cells_z: int,
    cell_size: float,
    origin: Sequence[float],
    youngs_modulus: float,
    compression_rate: float,
    compression_end_time: float,
    poisson_ratio: float = 0.3,
    density: float = 1.0e3,
) -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    """A slender FEM beam whose two pinned end faces are driven toward each other.

    Opposing scripted ``DeformableDirichletBoundaryCondition``s translate the end
    faces inward (along the bar's long x axis) at ``compression_rate`` until
    ``compression_end_time``; the soft FEM core buckles and folds onto itself, so
    its surface comes into contact with itself. This exercises the self-contact
    barrier on a volumetric FEM body -- a DART-native step toward the IPC paper's
    self-collision stress tests.
    """

    positions, tetrahedra, edges, node_index, (nx, ny, nz) = _fem_bar_mesh(
        cells_x, cells_y, cells_z, cell_size, origin
    )
    options = _fem_bar_options(
        positions, tetrahedra, youngs_modulus, poisson_ratio, density
    )

    axis_y = origin[1] + 0.5 * cells_y * cell_size
    axis_z = origin[2] + 0.5 * cells_z * cell_size

    def end_face(i: int) -> list[int]:
        return [node_index(i, j, k) for k in range(nz) for j in range(ny)]

    conditions = []
    for face_i, sign in ((0, 1.0), (nx - 1, -1.0)):
        condition = sx.DeformableDirichletBoundaryCondition()
        condition.nodes = end_face(face_i)
        condition.linear_velocity = np.array([sign * compression_rate, 0.0, 0.0])
        condition.center = np.array([origin[0] + face_i * cell_size, axis_y, axis_z])
        condition.start_time = 0.0
        condition.end_time = compression_end_time
        conditions.append(condition)
    options.dirichlet_boundary_conditions = conditions
    return options, edges


def build_cloth_from_obj(
    path: "str | Path",
    *,
    mass: float = 0.02,
    edge_stiffness: float = 200.0,
    damping: float = 1.0,
    translate: Sequence[float] = (0.0, 0.0, 0.0),
    fixed_nodes: Iterable[int] = (),
) -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    """Build a mass-spring cloth membrane from a Wavefront ``.obj`` surface mesh.

    Loads the triangle mesh via :func:`sx.load_obj_triangle_mesh`, then derives
    the mass-spring topology the World solver needs: one structural spring
    per unique triangle edge (rest length = the loaded inter-node distance), a
    uniform nodal ``mass``, and the surface triangles (kept for self-contact and
    rendering). ``translate`` offsets every vertex (e.g. to lift the sheet above
    an obstacle). Returns the options plus the unique edge list for the wireframe.
    """

    options = sx.load_obj_triangle_mesh(str(path))

    offset = np.asarray(translate, dtype=float)
    positions = [np.asarray(p, dtype=float) + offset for p in options.positions]
    options.positions = positions
    options.masses = [mass] * len(positions)
    options.edge_stiffness = edge_stiffness
    options.damping = damping

    seen: set[tuple[int, int]] = set()
    edges: list[tuple[int, int]] = []
    edge_objects: list[sx.DeformableEdge] = []
    for triangle in options.surface_triangles:
        corners = (triangle.node_a, triangle.node_b, triangle.node_c)
        for a, b in (
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[0]),
        ):
            key = (a, b) if a < b else (b, a)
            if key in seen:
                continue
            seen.add(key)
            rest = float(np.linalg.norm(positions[key[0]] - positions[key[1]]))
            edge_objects.append(sx.DeformableEdge(key[0], key[1], rest))
            edges.append(key)
    options.edges = edge_objects

    fixed = list(fixed_nodes)
    if fixed:
        options.fixed_nodes = fixed
    return options, edges


def build_strand_from_seg(
    path: "str | Path",
    *,
    mass: float = 0.05,
    edge_stiffness: float = 150.0,
    damping: float = 1.0,
    translate: Sequence[float] = (0.0, 0.0, 0.0),
    fixed_nodes: Iterable[int] = (),
) -> tuple["sx.DeformableBodyOptions", list[tuple[int, int]]]:
    """Build a mass-spring strand from a ``.seg`` segment mesh.

    Loads the segment mesh via :func:`sx.load_seg_line_mesh` (positions + spring
    edges with auto rest lengths), then sets a uniform nodal ``mass``,
    ``edge_stiffness`` and ``damping``. ``translate`` offsets every vertex.
    Returns the options plus the segment list for the wireframe.
    """

    options = sx.load_seg_line_mesh(str(path))
    offset = np.asarray(translate, dtype=float)
    options.positions = [
        np.asarray(p, dtype=float) + offset for p in options.positions
    ]
    options.masses = [mass] * len(options.positions)
    options.edge_stiffness = edge_stiffness
    options.damping = damping
    edges = [(edge.node_a, edge.node_b) for edge in options.edges]
    fixed = list(fixed_nodes)
    if fixed:
        options.fixed_nodes = fixed
    return options, edges


def build_particles_from_pt(
    path: "str | Path",
    *,
    mass: float = 0.02,
    translate: Sequence[float] = (0.0, 0.0, 0.0),
) -> "sx.DeformableBodyOptions":
    """Build a cloud of free deformable particles from a ``.pt`` point set.

    Loads the points via :func:`sx.load_point_set` (positions only) and assigns
    a uniform nodal ``mass``; with no springs or tetrahedra the nodes are inertial
    particles that fall under gravity and stack on contact barriers.
    """

    options = sx.load_point_set(str(path))
    offset = np.asarray(translate, dtype=float)
    options.positions = [
        np.asarray(p, dtype=float) + offset for p in options.positions
    ]
    options.masses = [mass] * len(options.positions)
    return options


class IpcDeformableBridge:
    """Mirrors deformable bodies onto a render `dart.simulation.World`."""

    def __init__(
        self, physics_world: Any, name: str = "ipc_deformable_render"
    ) -> None:
        self._physics_world = physics_world
        self._name = name
        # Render world is the classic World, retained for the Filament viewer.
        self.render_world = dart.gui.RenderWorld(name)
        self.render_world.set_gravity([0.0, 0.0, 0.0])
        self.render_world.set_time_step(getattr(physics_world, "time_step", 0.005))
        # (body, [node SimpleFrame, ...], edge LineSegmentShape) per deformable.
        self._deformables: list[tuple[Any, list[Any], Any]] = []
        self._step_failed = False
        self._min_height_history: deque[float] = deque(maxlen=120)

    def add_deformable_visual(
        self,
        body: Any,
        name: str = "deformable",
        edges: Iterable[tuple[int, int]] | None = None,
    ) -> None:
        """Create per-node spheres + an edge wireframe for ``body``.

        ``edges`` overrides the wireframe connectivity; when ``None`` the body's
        own spring edges are used. FEM bodies carry tetrahedra rather than spring
        edges, so they pass an explicit tet-derived wireframe.
        """

        node_count = body.node_count

        edge_shape = dart.LineSegmentShape(_EDGE_THICKNESS)
        for i in range(node_count):
            edge_shape.add_vertex(np.asarray(body.node_position(i), dtype=float))
        if edges is None:
            edges = [
                (body.edge(i).node_a, body.edge(i).node_b)
                for i in range(body.edge_count)
            ]
        for node_a, node_b in edges:
            edge_shape.add_connection(int(node_a), int(node_b))
        edge_frame = dart.SimpleFrame(dart.Frame.world(), f"{name}_edges", np.eye(4))
        edge_frame.set_shape(edge_shape)
        edge_frame.create_visual_aspect().set_color(list(_EDGE_COLOR))
        self.render_world.add_simple_frame(edge_frame)

        node_frames: list[Any] = []
        for i in range(node_count):
            fixed = body.is_fixed_node(i)
            frame = dart.SimpleFrame(
                dart.Frame.world(),
                f"{name}_node_{i}",
                _translation(body.node_position(i)),
            )
            frame.set_shape(
                dart.SphereShape(_FIXED_NODE_RADIUS if fixed else _FREE_NODE_RADIUS)
            )
            frame.create_visual_aspect().set_color(
                list(_FIXED_NODE_COLOR if fixed else _FREE_NODE_COLOR)
            )
            self.render_world.add_simple_frame(frame)
            node_frames.append(frame)

        self._deformables.append((body, node_frames, edge_shape))

    def add_rigid_box_visual(
        self,
        center: Sequence[float],
        full_extents: Sequence[float],
        color: tuple[float, float, float],
        name: str = "obstacle",
    ) -> None:
        """A static box SimpleFrame (e.g. a ground barrier or drape obstacle)."""

        frame = dart.SimpleFrame(dart.Frame.world(), name, _translation(center))
        frame.set_shape(dart.BoxShape(np.asarray(full_extents, dtype=float)))
        frame.create_visual_aspect().set_color(list(color))
        self.render_world.add_simple_frame(frame)

    def add_rigid_sphere_visual(
        self,
        center: Sequence[float],
        radius: float,
        color: tuple[float, float, float],
        name: str = "sphere_obstacle",
    ) -> None:
        """A static sphere SimpleFrame (e.g. a deformable obstacle barrier)."""

        frame = dart.SimpleFrame(dart.Frame.world(), name, _translation(center))
        frame.set_shape(dart.SphereShape(float(radius)))
        frame.create_visual_aspect().set_color(list(color))
        self.render_world.add_simple_frame(frame)

    def sync(self) -> None:
        """Copy every deformable node position onto its sphere + wireframe."""

        for body, node_frames, edge_shape in self._deformables:
            for i, frame in enumerate(node_frames):
                position = np.asarray(body.node_position(i), dtype=float)
                frame.set_transform(_translation(position))
                edge_shape.set_vertex(i, position)

    def _node_positions(self) -> list[np.ndarray]:
        positions: list[np.ndarray] = []
        for body, _node_frames, _edge_shape in self._deformables:
            for i in range(int(body.node_count)):
                positions.append(np.asarray(body.node_position(i), dtype=float))
        return positions

    def _fixed_node_count(self) -> int:
        fixed = 0
        for body, _node_frames, _edge_shape in self._deformables:
            for i in range(int(body.node_count)):
                try:
                    fixed += int(bool(body.is_fixed_node(i)))
                except Exception:  # noqa: BLE001
                    pass
        return fixed

    def build_diagnostics_panel(self, builder: Any, context: Any) -> None:
        """Render generic IPC deformable solver diagnostics."""

        del context
        node_count = sum(
            int(body.node_count) for body, _frames, _edge in self._deformables
        )
        positions = self._node_positions()
        builder.text("solver: deformable IPC")
        builder.text(f"world time: {self._physics_world.time:.3f} s")
        builder.text(f"time step: {self._physics_world.time_step:.4f} s")
        builder.text(f"nodes: {node_count} | fixed: {self._fixed_node_count()}")
        diagnostics = getattr(
            self._physics_world, "last_deformable_solver_diagnostics", None
        )
        if diagnostics is not None:
            builder.text(
                f"iters: {diagnostics.solver_iterations} | "
                f"line search: {diagnostics.line_search_trials}"
            )
            builder.text(
                f"self contacts: {diagnostics.self_contact_barrier_active_contacts} | "
                f"converged: {diagnostics.converged_active_contact_count}"
            )
            min_distance = float(diagnostics.min_active_contact_distance)
            if np.isfinite(min_distance):
                builder.text(f"min active distance: {min_distance:.5f} m")
        if positions:
            stacked = np.vstack(positions)
            minimum = np.min(stacked, axis=0)
            maximum = np.max(stacked, axis=0)
            self._min_height_history.append(float(minimum[2]))
            builder.text(f"z range: {minimum[2]:.3f} .. {maximum[2]:.3f} m")
            builder.text(
                f"span xy: {maximum[0] - minimum[0]:.3f} x "
                f"{maximum[1] - minimum[1]:.3f} m"
            )
        builder.text(f"step failed: {'yes' if self._step_failed else 'no'}")
        if self._min_height_history:
            builder.plot_lines("Min z", list(self._min_height_history))

    def pre_step(self) -> None:
        """Advance deformable physics one step, then sync the render.

        A solver failure (e.g. a divergent stiff-barrier solve) is reported
        once to stderr rather than silently swallowed, so a wedged demo is
        observable; subsequent frames keep rendering the last good state.
        """

        try:
            self._physics_world.step()
        except Exception as error:  # noqa: BLE001
            if not self._step_failed:
                self._step_failed = True
                print(
                    f"[ipc-deformable] {self._name}: solver step failed, "
                    f"freezing physics: {error}",
                    file=sys.stderr,
                )
        self.sync()


# Re-export math for scene modules that shape their grids with trig.
__all__ = [
    "IpcDeformableBridge",
    "build_fem_bar",
    "build_fem_twist_bar",
    "build_grid_options",
    "grid_index",
    "math",
]
