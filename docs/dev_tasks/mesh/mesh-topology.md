# Mesh Topology Preservation (Triangles vs Quads)

## Status

In progress. Polygon mesh storage, loader triangulation, and unit coverage are
in place; remaining work focuses on wiring polygon meshes into rendering paths
and validating downstream usage.

## Goals

- Preserve original polygon topology (quads/ngons) for rendering and export.
- Keep triangle meshes as the canonical collision/physics representation.
- Ensure loaders triangulate non-triangular faces instead of dropping them.

## Decisions

- TriMesh remains the canonical geometry for collision backends.
- A polygon mesh container stores variable-length faces and can triangulate
  deterministically for collision usage.
- Rendering continues to use Assimp scene data until the Assimp deprecation
  path is complete.
- Polygon-aware imports avoid Assimp pre-triangulation so face sizes are
  preserved for rendering/export.
- Triangulation uses ear clipping on a projected plane; faces are assumed
  simple and planar (concave supported).

## Plan

1. Add a polygon mesh container in `dart::math` with a deterministic
   triangulation routine that yields a `TriMesh`.
2. Update mesh loaders to populate polygon faces and build a triangulated
   `TriMesh` (no skipped faces).
3. Add unit coverage for quad triangulation and non-tri face handling.
4. Expose optional polygon mesh accessors in `MeshShape` and document the
   rendering/physics split.
