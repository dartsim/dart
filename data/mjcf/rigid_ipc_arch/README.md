# Deterministic 101-stone masonry arch source geometry

The 101 masonry stones derive from the pinned source parameters. This
directory retains the source license and provenance record.

The geometry is source-bound to the public FBF repository,
<https://github.com/matthcsong/fbf-sca-2026>, commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, path
`meshes/arch/num_stones=101/`. That repository credits the
[Rigid-IPC dataset](https://github.com/ipc-sim/rigid-ipc). The independent
upstream pin remains Rigid-IPC commit
`23b6ba6fbf8434056444ae106356fd2209136988` ("Write GLTF in input orientation",
2025-06-13). Rigid-IPC implements Ferguson et al., "Intersection-free Rigid
Body Dynamics" (SIGGRAPH 2021). Its MIT license is retained in
[`LICENSE.md`](LICENSE.md).

## Reproduction contract

The pinned source geometry uses the weighted-catenary parameters and
operations used by the source:

- `fc=60 cm`, `Qb=100 cm^2`, `Qt=49 cm^2`, and `L=30 cm`;
- composite Simpson integration and equal-arc-length bisection;
- constant per-stone square cross sections, source offsets, springer
  flattening, and the `0.1 cm` height normalization;
- source OBJ vertex order, twelve face triangles, and six-decimal coordinate
  quantization; and
- the source y/z-to-z-up axis rotation, `0.01` cm-to-m scale, uniform `0.5`
  friction, `0.005 s` timestep, and `9.8 m/s^2` gravity.

The ordered 2,424-coordinate inventory is pinned by FNV-1a64
`0x528596c9206aef89`, the same digest used by the DART Figure 8 construction
test. Before removing the vendored files, an independent audit proved:

- all 101 generated OBJ byte streams exactly matched the pinned copies;

Unit tests keep the inventory digest, representative vertices, self-contained
MJCF structure, and absence of file-backed mesh references fail-closed.

## Scope and limitations

This directory retains source provenance for example geometry. No core DART target reads it
and it adds no core dependency. The credited source MJCF records Rigid-IPC's MJCF port semantics; it is not a scene authored by the FBF paper and does not
establish historical paper parity. Density remains the source scene's MuJoCo per-mesh density default
(`1000 kg/m^3`), matching Rigid-IPC's unspecified-density default. All 101
stones remain dynamic and only the ground is fixed, matching the source JSON
but differing from DART's current Figure 8 adapter, which fixes both springers.
