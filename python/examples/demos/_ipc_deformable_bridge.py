"""Shared bridge + grid builder for the experimental IPC deformable demos.

The Filament viewer renders a `dart.simulation.World`, not the experimental
`dartpy.simulation_experimental` (sx) world that owns the deformable physics.
`_sx_bridge.SxRenderBridge` mirrors *rigid* sx bodies onto SimpleFrames by
syncing a single transform per body. Deformable bodies instead move every
node independently, so this bridge mirrors each deformable as:

- one `dart.SphereShape` SimpleFrame per node (orange for fixed/scripted nodes,
  blue for free nodes), its transform synced from ``body.node_position(i)``;
- one `dart.LineSegmentShape` SimpleFrame whose vertices track the nodes and
  whose connections are the body's spring edges (a wireframe of the mesh).

This is the Python equivalent of the C++ ``appendDeformableVisual`` /
``syncRenderFrames`` layers in
``examples/demos/scenes/experimental_deformable.cpp`` (per-node spheres + edge
wireframe). The dynamic *surface mesh* layer (``makeDeformableSurfaceRenderable``)
is not reachable through ``dartpy.gui.run_demos`` today, so it is intentionally
omitted; the spheres + wireframe convey sag, drape, and scripted motion.

Scenes build a deformable grid with :func:`build_grid_options`, add it to an
sx world, then register it with an :class:`IpcDeformableBridge` and return
``SceneSetup(world=bridge.render_world, pre_step=bridge.pre_step, ...)``.
"""

from __future__ import annotations

import math
import sys
from typing import Any, Callable, Iterable, Sequence

import dartpy as dart
import dartpy.simulation_experimental as sx
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

    nx, ny, nz = cells_x + 1, cells_y + 1, cells_z + 1

    def node_index(i: int, j: int, k: int) -> int:
        return i + nx * (j + ny * k)

    positions: list[Sequence[float]] = []
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                positions.append(
                    (
                        origin[0] + i * cell_size,
                        origin[1] + j * cell_size,
                        origin[2] + k * cell_size,
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

    fixed = [node_index(0, j, k) for k in range(nz) for j in range(ny)]

    options = sx.DeformableBodyOptions()
    options.positions = [np.asarray(p, dtype=float) for p in positions]
    options.tetrahedra = tetrahedra
    options.fixed_nodes = fixed
    options.material.youngs_modulus = youngs_modulus
    options.material.poisson_ratio = poisson_ratio
    options.material.density = density
    options.material.use_finite_element_elasticity = True
    return options, sorted(edges)


class IpcDeformableBridge:
    """Mirrors sx deformable bodies onto a render `dart.simulation.World`."""

    def __init__(self, sx_world: Any, name: str = "ipc_deformable_render") -> None:
        self._sx_world = sx_world
        self._name = name
        self.render_world = dart.World(name)
        self.render_world.set_gravity([0.0, 0.0, 0.0])
        self.render_world.set_time_step(getattr(sx_world, "time_step", 0.005))
        # (body, [node SimpleFrame, ...], edge LineSegmentShape) per deformable.
        self._deformables: list[tuple[Any, list[Any], Any]] = []
        self._step_failed = False

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
            edge_shape.addVertex(np.asarray(body.node_position(i), dtype=float))
        if edges is None:
            edges = [
                (body.edge(i).node_a, body.edge(i).node_b)
                for i in range(body.edge_count)
            ]
        for node_a, node_b in edges:
            edge_shape.addConnection(int(node_a), int(node_b))
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

    def sync(self) -> None:
        """Copy every deformable node position onto its sphere + wireframe."""

        for body, node_frames, edge_shape in self._deformables:
            for i, frame in enumerate(node_frames):
                position = np.asarray(body.node_position(i), dtype=float)
                frame.set_transform(_translation(position))
                edge_shape.setVertex(i, position)

    def pre_step(self) -> None:
        """Advance the sx deformable physics one step, then sync the render.

        A solver failure (e.g. a divergent stiff-barrier solve) is reported
        once to stderr rather than silently swallowed, so a wedged demo is
        observable; subsequent frames keep rendering the last good state.
        """

        try:
            self._sx_world.step()
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
    "build_grid_options",
    "grid_index",
    "math",
]
